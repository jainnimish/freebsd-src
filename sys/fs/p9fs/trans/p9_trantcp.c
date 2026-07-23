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
#include <sys/endian.h>
#include <sys/errno.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/sx.h>
#include <sys/sysctl.h>
#include <sys/socket.h>
#include <sys/socketvar.h>
#include <sys/types.h>
#include <sys/queue.h>
#include <sys/uio.h>

#include <netinet/in.h>
#include <netinet/tcp.h>

#include <fs/p9fs/p9_client.h>
#include <fs/p9fs/p9_transport.h>

#define IN9P_SX(_sc) (&(_sc)->in9p_sx)
#define IN9P_XLOCK(_sc) sx_xlock(IN9P_SX(_sc))
#define IN9P_XUNLOCK(_sc) sx_xunlock(IN9P_SX(_sc))
#define IN9P_LOCK_INIT(_sc) sx_init(IN9P_SX(_sc), \
    "INET 9P CHAN lock")
#define IN9P_LOCK_DESTROY(_sc) sx_destroy(IN9P_SX(_sc))
static MALLOC_DEFINE(M_IN9P_TRANS, "in9p_sc", "P9 INET transport");

struct in9p_sc {
	struct socket *so;
	struct sockaddr_in sin;
	struct sx in9p_sx;
	STAILQ_ENTRY(in9p_sc) chan_next;
};

/* Global channel list. Each channel will correspond to a mount point. */
static STAILQ_HEAD( ,in9p_sc) global_chan_list =
    STAILQ_HEAD_INITIALIZER(global_chan_list);
static struct mtx global_chan_list_mtx;
MTX_SYSINIT(global_chan_list_mtx, &global_chan_list_mtx, "9p INET global", MTX_DEF);

/* So we don't unload the module if this transport is registered. */
static int trans_loaded = 0;

/*
 * Maximum number of seconds in9p_request thread sleep waiting for an
 * ack from the host, before exiting.
 */
SYSCTL_NODE(_vfs, OID_AUTO, 9p, CTLFLAG_RW, 0, "9P File System Protocol");

static unsigned int in9p_ackmaxidle = 15;
SYSCTL_UINT(_vfs_9p, OID_AUTO, inmaxidle, CTLFLAG_RW, &in9p_ackmaxidle, 0,
    "Maximum time to wait for P9 server to respond");

/* No way to find out what clnt->msize is for now. */
#define P9_MTU 131072
#define IN9P_SORECEIVE_CHUNK (1024 * 16)

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

	bzero(&sopt, sizeof(struct sockopt));
	ts.tv_sec = in9p_ackmaxidle;
	ts.tv_usec = 0;
	sopt.sopt_dir = SOPT_SET;
	sopt.sopt_level = SOL_SOCKET;
	sopt.sopt_name = SO_RCVTIMEO;
	sopt.sopt_val = &ts;
	sopt.sopt_valsize = sizeof(ts);
	if ((error = sosetopt(so, &sopt)) != 0)
		return (error);

	sopt.sopt_name = SO_SNDTIMEO;
	if ((error = sosetopt(so, &sopt)) != 0)
		return (error);

	val = 1;
	sopt.sopt_level = IPPROTO_TCP;
	sopt.sopt_name = TCP_NODELAY;
	sopt.sopt_val = &val;
	sopt.sopt_valsize = sizeof(val);
	if ((error = sosetopt(so, &sopt)) != 0)
		return (error);

	val = 1;
	sopt.sopt_level = SOL_SOCKET;
	sopt.sopt_name = SO_KEEPALIVE;
	if ((error = sosetopt(so, &sopt)) != 0)
		return (error);

	if ((error = soconnect(so, sintosa(&chan->sin), curthread)) != 0)
		return (error);

	SOCK_LOCK(so);
	while ((so->so_state & SS_ISCONNECTING) && so->so_error == 0) {
		error = msleep(&so->so_timeo, SOCK_MTX(so), PSOCK | PCATCH,
		    "in9p_connect", 0);
		if (error != 0 && (so->so_state & SS_ISCONNECTING) && so->so_error == 0) {
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

	/*
	 * Since each client gets its own socket,
	 * we support having multiple channels with the same sharename.
	 */
	chan = malloc(sizeof(struct in9p_sc), M_IN9P_TRANS, M_WAITOK | M_ZERO);
	chan->so = NULL;
	sin = &chan->sin;

	/* TODO: Deal with AF_INET6 */
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
		goto err;
	}
	STAILQ_INSERT_HEAD(&global_chan_list, chan, chan_next);
	mtx_unlock(&global_chan_list_mtx);
	IN9P_LOCK_INIT(chan);

	*handlep = (void *)chan;
	free(host, M_TEMP);

	return (0);

err:
	if (chan->so != NULL) {
		soclose(chan->so);
		chan->so = NULL;
	}
	free(host, M_TEMP);
	free(chan, M_IN9P_TRANS);
	return (error);
}

/*
 * This is called after vflush and TCLUNKs are done.
 * in9p_request() should not be called anymore.
 */
static void
in9p_close(void *handle)
{
	struct in9p_sc *chan = handle;

	mtx_lock(&global_chan_list_mtx);
	STAILQ_REMOVE(&global_chan_list, chan, in9p_sc, chan_next);
	mtx_unlock(&global_chan_list_mtx);
	soclose(chan->so);
	IN9P_LOCK_DESTROY(chan);
	free(chan, M_IN9P_TRANS);
}

/*
 * Avoid reading stale data by creating a new socket.
 * NB: Caller must hold channel lock.
 */
static inline void
in9p_sock_clean(struct in9p_sc *chan, int *error)
{
	soclose(chan->so);
	chan->so = NULL;

	switch ((*error = in9p_sock_create(chan))) {
	case (0):
		break;
	case (EHOSTUNREACH):
	case (ECONNREFUSED):
	case (EHOSTDOWN):
	case (ENETDOWN):
		break;
	default:
		/* Bail for now. */
		panic("INET P9 transport: failed to revive connection");
	}
}

/* Helper function to reset common uio fields. */
static inline void
in9p_reset_uio(struct uio *auio, struct iovec *iov)
{
	auio->uio_iov = iov;
	auio->uio_iovcnt = 1;
	auio->uio_offset = 0;
	auio->uio_resid = iov->iov_len;
	auio->uio_td = curthread;
}

static int
in9p_sock_read(struct socket *so, struct uio *auio, int *flags)
{
	int error;

	do {
		*flags = MSG_WAITALL;
		error = soreceive(so, NULL, auio, NULL, NULL, flags);
	} while (error == EINTR || error == ERESTART);

	/* Treat all other errors, including EWOULDBLOCK, as fatal. */
	if (error)
		return (error);
	if (auio->uio_resid > 0)
		return (EPIPE);

	return (0);
}

/*
 * Request handler. This is called for every request submitted to the host.
 * Since the server sends responses on a STREAM socket without any locks,
 * we preemptively serialize requests to prevent intermingling of P9 packets.
 */
static int
in9p_request(void *handle, struct p9_req_t *req)
{
	struct in9p_sc *chan = handle;
	struct socket *so = chan->so;
	struct iovec iov;
	struct uio auio;
	size_t len;
	int flags, error = 0;

	IN9P_XLOCK(chan);
	/*
	 * TODO: Add conditional statements to p9fs_doio and p9fs_write to
	 * pass the uiov for TWRITE instead of copying it out to io_buffer.
	 */
	auio.uio_segflg = UIO_SYSSPACE;
	auio.uio_rw = UIO_WRITE;
	iov.iov_base = req->tc.sdata;
	iov.iov_len = req->tc.size;
	in9p_reset_uio(&auio, &iov);
	if ((error = sosend(so, NULL, &auio, NULL, NULL, 0, curthread)) > 0 ||
	   auio.uio_resid > 0)
		goto clean;

	/* Read the response size. */
	auio.uio_rw = UIO_READ;
	iov.iov_base = req->rc.sdata;
	iov.iov_len = sizeof(req->rc.size);
	in9p_reset_uio(&auio, &iov);
	if ((error = in9p_sock_read(so, &auio, &flags)) != 0)
		goto clean;
	req->rc.size = le32dec(req->rc.sdata);

	/* For the same reasons as SMB, we choose to receive chunks. */
	len = req->rc.size - sizeof(req->rc.size);
	while (len > 0) {
		iov.iov_len = MIN(len, IN9P_SORECEIVE_CHUNK);
		in9p_reset_uio(&auio, &iov);
		len -= iov.iov_len;
		if ((error = in9p_sock_read(so, &auio, &flags)) != 0)
			goto clean;

	}
	IN9P_XUNLOCK(chan);

	return (error);
clean:
	in9p_sock_clean(chan, &error);
	IN9P_XUNLOCK(chan);
	return (error ? error : EPIPE);
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
