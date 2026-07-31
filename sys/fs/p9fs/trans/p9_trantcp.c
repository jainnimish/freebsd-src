/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Nimish Jain
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */
/*
 * The INET 9P transport driver.
 */

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/kthread.h>
#include <sys/endian.h>
#include <sys/errno.h>
#include <sys/mbuf.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/proc.h>
#include <sys/protosw.h>
#include <sys/sysctl.h>
#include <sys/systm.h>
#include <sys/socket.h>
#include <sys/socketvar.h>
#include <sys/types.h>
#include <sys/queue.h>
#include <sys/uio.h>

#include <netinet/in.h>
#include <netinet/tcp.h>
#include <net/vnet.h>

#include <fs/p9fs/p9_client.h>
#include <fs/p9fs/p9_transport.h>

#define IN9P_RX_MTX(_sc) (&(_sc)->in9p_rx_mtx)
#define IN9P_RX_LOCK(_sc) mtx_lock(IN9P_RX_MTX(_sc))
#define IN9P_RX_UNLOCK(_sc) mtx_unlock(IN9P_RX_MTX(_sc))
#define IN9P_RX_INIT(_sc) mtx_init(IN9P_RX_MTX(_sc), \
    "IN9P receive-queue mutex", NULL, MTX_DEF)
#define IN9P_RX_DESTROY(_sc) mtx_destroy(IN9P_RX_MTX(_sc))

static MALLOC_DEFINE(M_IN9P_TRANS, "in9p_sc", "P9 INET transport");

struct p9_header {
	uint32_t size;
	uint8_t type;
	uint16_t tag;
} __packed;

/* States for the request-receive state machine. */
#define IN9P_QUEUED		(1 << 0)
#define IN9P_INPROGRESS		(1 << 1)
#define IN9P_DONE		(1 << 2)
#define IN9P_DRAINED		(1 << 3)

/* Tasks for the request queue. */
struct in9p_task {
	struct p9_req_t *req;
	int status;
	int error;
	STAILQ_ENTRY(in9p_task) tk_next;
};

struct in9p_sc {
	bool broken;			/* Accepting new requests? */
	bool discon;			/* Teardown? */
	bool wake;			/* Wake up request thread? */
	bool td_running;		/* Receive thread running? */
	size_t nreq;			/* Number of pending requests. */
	struct socket *so;
	struct sockaddr_in sin;
	STAILQ_HEAD(,in9p_task) rxq;	/* Receive queue. */
	struct mtx in9p_rx_mtx;		/* Mutex for receive queue. */
	struct cv in9p_rx_cv;		/* Receive thread cond var. */
	struct cv in9p_drain_ack;	/* Drained requests cond var. */
	STAILQ_ENTRY(in9p_sc) chan_next;
};

/* Global channel list. Each channel will correspond to a mount point. */
static STAILQ_HEAD(,in9p_sc) global_chan_list =
    STAILQ_HEAD_INITIALIZER(global_chan_list);
static struct mtx global_chan_list_mtx;
MTX_SYSINIT(global_chan_list_mtx, &global_chan_list_mtx, "9p INET global", MTX_DEF);

/* So we do not unload the module if this transport is registered. */
static int trans_loaded = 0;

SYSCTL_DECL(_vfs_9p);
static unsigned int in9p_netmaxidle = 60;
SYSCTL_UINT(_vfs_9p, OID_AUTO, netmaxidle, CTLFLAG_RW, &in9p_netmaxidle, 0,
    "Maximum time to wait for P9 server to respond");

/* No way to find out what clnt->msize is for now. */
#define P9_MTU 131072
#define IN9P_SORECEIVE_CHUNK (1024 * 16)

static int
in9p_sock_upcall(struct socket *so, void *arg, int flags __unused)
{
	struct in9p_sc *chan = arg;

	if (soreadable(so) && chan->wake)
		cv_signal(&chan->in9p_rx_cv);

	return (SU_OK);
}

static int
in9p_sock_read(struct socket *so, struct uio *auio)
{
	int error, flags;

	do {
		flags = MSG_WAITALL;
		error = soreceive(so, NULL, auio, NULL, NULL, &flags);
	} while (error == EINTR || error == ERESTART);

	/* Treat all other errors, including EWOULDBLOCK, as fatal. */
	if (error != 0)
		return (error);
	if (auio->uio_resid > 0)
		return (EPIPE);

	return (0);
}

/* Helper function to reset common uio fields. */
static inline void
in9p_reset_uio(struct uio *auio, struct iovec *iov)
{
	auio->uio_iov = iov;
	auio->uio_iovcnt = 1;
	auio->uio_offset = 0;
	auio->uio_resid = iov->iov_len;
}

static struct in9p_task *
in9p_fetch_tag(struct in9p_sc *chan, struct p9_header *hdr)
{
	struct in9p_task *tk;

	IN9P_RX_LOCK(chan);
	STAILQ_FOREACH(tk, &chan->rxq, tk_next) {
		if (tk->req->tc.tag == hdr->tag) {
			tk->status = IN9P_INPROGRESS;
			IN9P_RX_UNLOCK(chan);
			return (tk);
		}
	}
	IN9P_RX_UNLOCK(chan);

	return (NULL);
}

static void
in9p_drain_req(struct in9p_sc *chan, int error)
{
	struct in9p_task *tk;

	IN9P_RX_LOCK(chan);
	chan->broken = 1;
	STAILQ_FOREACH(tk, &chan->rxq, tk_next) {
		tk->error = (error ? error : ECONNABORTED);
		tk->status = IN9P_DONE | IN9P_DRAINED;
		wakeup(&tk->req->rc.tag);
	}
	STAILQ_INIT(&chan->rxq);

	/* Wait till final acknowledgement. */
	while (chan->nreq != 0)
		cv_wait(&chan->in9p_drain_ack, IN9P_RX_MTX(chan));
	IN9P_RX_UNLOCK(chan);
}

static void
in9p_receive_thread(void *arg)
{
	struct in9p_sc *chan = (struct in9p_sc *)arg;
	struct socket *so = chan->so;
	struct p9_header hdr;
	struct in9p_task *tk;
	struct p9_req_t *req;
	struct iovec iov;
	struct uio auio;
	size_t len;
	int error = 0;

	CURVNET_SET(so->so_vnet);

	for (;;) {
		SOCK_RECVBUF_LOCK(so);
		for (;;) {
			if (chan->discon) {
				SOCK_RECVBUF_UNLOCK(so);
				goto end;
			}

			/*
			 * We don't and can't check for chan->broken here.
			 * If in9p_request() calls sodisconnect(), eventually
			 * socantrcvmore() will be called, and we will get here.
			 */
			if (so->so_error)
				error = so->so_error;
			else if (so->so_rerror)
				error = so->so_rerror;
			else if (so->so_rcv.sb_state & SBS_CANTRCVMORE)
				error = EPIPE;

			/* Only wake up when in9p_close() is called now. */
			if (error != 0) {
				chan->wake = 0;
				SOCK_RECVBUF_UNLOCK(so);
				in9p_drain_req(chan, error);
				SOCK_RECVBUF_LOCK(so);
				if (chan->discon) {
					SOCK_RECVBUF_UNLOCK(so);
					goto end;
				}
			} else if (sbavail(&(so)->so_rcv) >= so->so_rcv.sb_lowat) {
				chan->wake = 0;
				m_copydata(so->so_rcv.sb_mb, 0, sizeof(hdr), (void *)&hdr);
				SOCK_RECVBUF_UNLOCK(so);
				break;
			} else {
				chan->wake = 1;
			}
			cv_wait(&chan->in9p_rx_cv, SOCK_RECVBUF_MTX(so));
		}

		/* Could be a timeout request, so kill. */
		if ((tk = in9p_fetch_tag(chan, &hdr)) == NULL) {
			(void) sodisconnect(so);
			error = EBADMSG;
			continue;
		}

		req = tk->req;
		auio.uio_segflg = UIO_SYSSPACE;
		auio.uio_rw = UIO_READ;
		auio.uio_td = curthread;
		iov.iov_base = req->rc.sdata;
		iov.iov_len = sizeof(req->rc.size);
		in9p_reset_uio(&auio, &iov);
		if ((error = in9p_sock_read(so, &auio)) != 0 ||
	            (req->rc.size = le32dec(req->rc.sdata)) > P9_MTU ||
		    (req->rc.size < sizeof(hdr))) {
			if (error == 0)
				error = EINVAL;
			(void) sodisconnect(so);
			goto done;
		}

		/* For the same reasons as SMB, we choose to receive chunks. */
		len = req->rc.size - sizeof(req->rc.size);
		while (len > 0) {
			iov.iov_len = MIN(len, IN9P_SORECEIVE_CHUNK);
			in9p_reset_uio(&auio, &iov);
			len -= iov.iov_len;
			if ((error = in9p_sock_read(so, &auio)) != 0) {
				(void) sodisconnect(so);
				break;
			}
		}

	done:
		/* Wakeup whoseever tag it is. */
		IN9P_RX_LOCK(chan);
		tk->status = IN9P_DONE;
		tk->error = error;
		STAILQ_REMOVE(&chan->rxq, tk, in9p_task, tk_next);
		wakeup(&req->rc.tag);
		if (error != 0)
			chan->broken = 1;
		IN9P_RX_UNLOCK(chan);
	}
end:
	/* XXX- not sure if we can still have pending requests. */
	in9p_drain_req(chan, error);
	SOCK_RECVBUF_LOCK(so);
	chan->td_running = 0;
	cv_signal(&chan->in9p_rx_cv);
	SOCK_RECVBUF_UNLOCK(so);
	CURVNET_RESTORE();
	kthread_exit();
}

static int
in9p_sock_create(struct in9p_sc *chan)
{
	struct socket *so;
	struct sockopt sopt;
	struct timeval ts;
	int error, val;

	if ((error = socreate(PF_INET, &chan->so, SOCK_STREAM,
	    IPPROTO_TCP, curthread->td_ucred, curthread)) != 0)
		return (error);

	so = chan->so;
	if ((error = soreserve(so, P9_MTU, P9_MTU)) != 0)
		return (error);

	bzero(&sopt, sizeof(sopt));
	ts.tv_sec = in9p_netmaxidle;
	ts.tv_usec = 0;

	/* Set the socket options. */
	sopt.sopt_dir = SOPT_SET;
	sopt.sopt_level = SOL_SOCKET;
	sopt.sopt_name = SO_RCVTIMEO;
	sopt.sopt_val = &ts;
	sopt.sopt_valsize = sizeof(ts);
	if ((error = sosetopt(so, &sopt)) != 0)
		return (error);

	val = sizeof(struct p9_header);
	sopt.sopt_name = SO_RCVLOWAT;
	sopt.sopt_val = &val;
	sopt.sopt_valsize = sizeof(val);
	if ((error = sosetopt(so, &sopt)) != 0)
		return (error);

	val = 1;
	sopt.sopt_name = SO_KEEPALIVE;
	sopt.sopt_val = &val;
	sopt.sopt_valsize = sizeof(val);
	if ((error = sosetopt(so, &sopt)) != 0)
		return (error);

	/* Set the protocol options. */
	val = 1;
	sopt.sopt_level = IPPROTO_TCP;
	sopt.sopt_name = TCP_NODELAY;
	sopt.sopt_val = &val;
	sopt.sopt_valsize = sizeof(val);
	if ((error = sosetopt(so, &sopt)) != 0)
		return (error);

	if ((error = soconnect(so, sintosa(&chan->sin), curthread)) != 0)
		return (error);

	SOCK_LOCK(so);
	while ((so->so_state & SS_ISCONNECTING) && so->so_error == 0) {
		error = msleep(&so->so_timeo, SOCK_MTX(so), PSOCK | PCATCH,
		    "in9p_connect", 0);
		if (error != 0 && (so->so_state & SS_ISCONNECTING) &&
		    so->so_error == 0) {
			so->so_state &= ~SS_ISCONNECTING;
			break;
		}
	}
	if (so->so_error) {
		error = so->so_error;
		so->so_error = 0;
	}
	SOCK_UNLOCK(so);
	if (error)
		return (error);

	SOCK_RECVBUF_LOCK(so);
	soupcall_set(so, SO_RCV, in9p_sock_upcall, chan);
	SOCK_RECVBUF_UNLOCK(so);

	return (0);
}

/*
 * Allocate a new INET channel. This sets up a transport channel
 * for 9P communication.
 */
static int
in9p_create(const char *mount_tag, void **handlep)
{
	struct in9p_sc *chan;
	char *host, *port;
	struct sockaddr_in *sin;
	int error;

	chan = malloc(sizeof(struct in9p_sc), M_IN9P_TRANS, M_WAITOK | M_ZERO);
	IN9P_RX_INIT(chan);
	STAILQ_INIT(&chan->rxq);
	cv_init(&chan->in9p_rx_cv, "in9p_rx_cv");
	cv_init(&chan->in9p_drain_ack, "in9p_drain_ack");

	/* TODO: Deal with AF_INET6 */
	sin = &chan->sin;
	host = strdup(mount_tag, M_TEMP);
	if ((port = strrchr(host, ':')) == NULL) {
		error = EINVAL;
		goto err;
	}
	*port = '\0'; port++;

	sin->sin_len = sizeof(*sin);
	sin->sin_family = AF_INET;
	sin->sin_port = htons((uint16_t)strtol(port, NULL, 10));
	if (inet_pton(AF_INET, host, &sin->sin_addr) < 1) {
		error = EINVAL;
		goto err;
	}

	if ((error = in9p_sock_create(chan)) != 0)
		goto err;

	/* Stop race between kldunload and p9_client_create. */
	mtx_lock(&global_chan_list_mtx);
	if (!trans_loaded) {
		mtx_unlock(&global_chan_list_mtx);
		error = ENXIO;
		goto upclr;
	}
	STAILQ_INSERT_HEAD(&global_chan_list, chan, chan_next);
	mtx_unlock(&global_chan_list_mtx);

	/* Add the receive thread. */
	chan->td_running = 1;
	error = kthread_add(in9p_receive_thread, chan, NULL, NULL,
	    0, 0, "in9p_receive_thread");
	if (error)
		goto remove;

	free(host, M_TEMP);
	*handlep = (void *)chan;

	return (0);


remove:
	mtx_lock(&global_chan_list_mtx);
	STAILQ_REMOVE(&global_chan_list, chan, in9p_sc, chan_next);
	mtx_unlock(&global_chan_list_mtx);
upclr:
	SOCK_RECVBUF_LOCK(chan->so);
	soupcall_clear(chan->so, SO_RCV);
	SOCK_RECVBUF_UNLOCK(chan->so);
err:
	if (chan->so != NULL)
		soclose(chan->so);
	cv_destroy(&chan->in9p_rx_cv);
	cv_destroy(&chan->in9p_drain_ack);
	IN9P_RX_DESTROY(chan);
	free(host, M_TEMP);
	free(chan, M_IN9P_TRANS);
	return (error);
}

/*
 * This is called after vflush() and TCLUNKs are done.
 * XXX - Not sure if all requests must be completed by now.
 */
static void
in9p_close(void *handle)
{
	struct in9p_sc *chan = (struct in9p_sc *)handle;

	/* Shutdown receive thread. */
	SOCK_RECVBUF_LOCK(chan->so);
	chan->discon = 1;
	soupcall_clear(chan->so, SO_RCV);
	cv_signal(&chan->in9p_rx_cv);
	while (chan->td_running)
		cv_wait(&chan->in9p_rx_cv, SOCK_RECVBUF_MTX(chan->so));
	SOCK_RECVBUF_UNLOCK(chan->so);

	/* Free the socket. */
	soclose(chan->so);

	/* Finally un-init all other variables. */
	cv_destroy(&chan->in9p_rx_cv);
	cv_destroy(&chan->in9p_drain_ack);
	IN9P_RX_DESTROY(chan);
	free(chan, M_IN9P_TRANS);

	/* Remove the handler. */
	mtx_lock(&global_chan_list_mtx);
	STAILQ_REMOVE(&global_chan_list, chan, in9p_sc, chan_next);
	mtx_unlock(&global_chan_list_mtx);
}

/*
 * Request handler. This is called for every request submitted to the host.
 */
static int
in9p_request(void *handle, struct p9_req_t *req)
{
	struct in9p_sc *chan = handle;
	struct socket *so = chan->so;
	struct in9p_task tk;
	struct iovec iov;
	struct uio auio;
	int error = 0;

	/* Enqueue this request. */
	IN9P_RX_LOCK(chan);
	if (!chan->broken) {
		tk.req = req;
		tk.error = 0;
		STAILQ_INSERT_TAIL(&chan->rxq, &tk, tk_next);
		tk.status = IN9P_QUEUED;
		chan->nreq++;
	} else {
		IN9P_RX_UNLOCK(chan);
		return (ENOTCONN);
	}
	IN9P_RX_UNLOCK(chan);

	/*
	 * TODO: Add conditional statements to p9fs_doio and p9fs_write to
	 * pass the uiov for TWRITE instead of copying it out to io_buffer.
	 */
	auio.uio_segflg = UIO_SYSSPACE;
	auio.uio_rw = UIO_WRITE;
	auio.uio_td = curthread;
	iov.iov_base = req->tc.sdata;
	iov.iov_len = req->tc.size;
	in9p_reset_uio(&auio, &iov);
	error = sosend(so, NULL, &auio, NULL, NULL, 0, curthread);
	if ((error != 0 && error != ENOBUFS) || auio.uio_resid > 0)
		(void) sodisconnect(so);
	if (auio.uio_resid > 0)
		error = EPIPE;

	IN9P_RX_LOCK(chan);
	/* Response received before we could sleep. */
	if (tk.status & IN9P_DONE) {
		error = tk.error;
		goto ok;
	}

	/* Error from sosend(), request will never be in progress. */
	if (error != 0 && error != ENOBUFS)
		chan->broken = 1;
	if (error != 0)
		goto clean;

	for (;;) {
		error = msleep(&req->rc.tag, IN9P_RX_MTX(chan), PSOCK,
		   "in9p_request", hz * in9p_netmaxidle);
		if (tk.status & IN9P_DONE) {
			error = tk.error;
			goto ok;
		}
		if (error == EWOULDBLOCK && tk.status == IN9P_QUEUED)
			goto clean;
	}
clean:
	STAILQ_REMOVE(&chan->rxq, &tk, in9p_task, tk_next);
ok:
	if(--chan->nreq == 0 && (tk.status & IN9P_DRAINED))
		cv_signal(&chan->in9p_drain_ack);
	IN9P_RX_UNLOCK(chan);
	return (error);
}

static struct p9_trans_module in9p_trans = {
	.name = "inet",
	.create = in9p_create,
	.close = in9p_close,
	.request = in9p_request,
};

static int
in9p_modevent(module_t mod, int type, void *unused)
{
	int error = 0;

	switch (type) {
	case MOD_LOAD:
		mtx_lock(&global_chan_list_mtx);
		if (!trans_loaded) {
			p9_register_trans(&in9p_trans);
			trans_loaded = 1;
		}
		mtx_unlock(&global_chan_list_mtx);
		break;
	case MOD_UNLOAD:
		mtx_lock(&global_chan_list_mtx);
		if (STAILQ_EMPTY(&global_chan_list)) {
			p9_unregister_trans(&in9p_trans);
			trans_loaded = 0;
		} else
			error = EBUSY;
		mtx_unlock(&global_chan_list_mtx);
		break;
	case MOD_SHUTDOWN:
		break;
	default:
		error = EOPNOTSUPP;
		break;
	}

	return (error);
}

static moduledata_t in9p_mod = {
	"p9_trantcp",
	in9p_modevent,
	NULL,
};
DECLARE_MODULE(p9_trantcp, in9p_mod, SI_SUB_VFS, SI_ORDER_ANY);

MODULE_VERSION(p9_trantcp, 1);
MODULE_DEPEND(p9_trantcp, p9fs, 1, 1, 1);
