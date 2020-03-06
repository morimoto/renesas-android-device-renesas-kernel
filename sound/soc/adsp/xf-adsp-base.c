/** ***************************************************************************
 *\file		xf-adsp-base.c
 *\brief	Source file for ADSP Base Control layer
 *\addtogroup	ADSP Driver
 ******************************************************************************
 *\date		Oct. 21, 2017
 *\author	Renesas Electronics Corporation
 ******************************************************************************
 *\par		Copyright
 *
 * Copyright(c) 2016 Renesas Electoronics Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 ******************************************************************************/

#include <linux/slab.h>
#include <linux/kthread.h>
#include <linux/wait.h>

#include "xf-adsp-base.h"

#define XF_AUX_POOL_SIZE	(16)	/**< size of auxiliary pool*/
#define XF_AUX_POOL_MSG_LENGTH	(256)	/**< size of auxiliary buffer   */

#define XF_PROXY_ALIGN		(64)	/**< proxy alignment		*/

/** \def XF_ALIGNED(size)
 *  Get properly aligned buffer length
 */
#define XF_ALIGNED(size)\
((((size) + XF_PROXY_ALIGN) - 1) & ~(XF_PROXY_ALIGN - 1)) /* PRQA S 3453 */

#ifndef offset_of
/** \def offset_of(type, member)
 *  Return the offset of member in type structuer in byte
 */
#define offset_of(type, member)  \
	((int)(long int)&(((const type *)(0))->member))	/* PRQA S 3453 */
#endif

/*************************************************
 * Variable and and function declaration
 * **********************************************/

/** ADSP base control data */
static struct xf_adsp_base *base;

/* function declaration */
static int xf_adsp_base_register_handle(void *private_data,
					struct xf_callback_func *cb,
					int comp_id);
static inline struct xf_handle *xf_adsp_base_get_handle(int handle_id);
static int xf_adsp_base_free_handle(int handle_id);
static void xf_adsp_base_init_handle(void);
static int xf_adsp_base_get_valid_handle(void);
static int xf_send_and_receive(struct xf_message *msg);
static int xf_response_thread(void *data);
static void xf_buffer_put(struct xf_buffer *buffer);
static u32 xf_buffer_length(struct xf_buffer *b);
static struct xf_buffer *xf_buffer_get(struct xf_pool *pool);
static void *xf_buffer_data(struct xf_buffer *b);
static int xf_adsp_unregister(int comp_id);
static int xf_adsp_register(char *name, int *comp_id);

/*************************************************
 * Helper function to process pool data
 * **********************************************/

/** ***********************************************************
 *  \brief get buffer from given pool
 *  \internal
 *  \covers: DD_DRV_CMN_01_042
 *
 *  \param[in]  pool	Data pool address
 *  \retval	b	Pointer to buffer address in pool
 **************************************************************/
static struct xf_buffer *xf_buffer_get(struct xf_pool *pool)
{
	struct xf_buffer *b;

	b = pool->free;
	if (b) {
		pool->free = b->link.next;
		b->link.pool = pool;
	}
	return b;
}

/***********************************************************
 *\brief return buffer back to pool
 *\internal
 *\covers: DD_DRV_CMN_01_043
 *
 *\param[in]  buffer  Pointer to the buffer data
 ************************************************************/
static void xf_buffer_put(struct xf_buffer *buffer)
{
	struct xf_pool *pool = buffer->link.pool;

	buffer->link.next = pool->free;
	pool->free = buffer;
}

/***********************************************************
 *\brief get the address of the given buffer data
 *\internal
 *\covers: DD_DRV_CMN_01_044
 *
 *\param[in]	b		Pointer to the buffer data
 *\retval	address		Address of buffer data
 ************************************************************/
static void *xf_buffer_data(struct xf_buffer *b)
{
	return b->address;
}

/************************************************************
 *\brief get the length of the given buffer data
 *\internal
 *\covers: DD_DRV_CMN_01_045
 *
 *\param[in]	b		Pointer to the buffer data
 *\retval	length		Size of buffer data
 ************************************************************/
static u32 xf_buffer_length(struct xf_buffer *b)  /* PRQA S 3673 */
{
	return b->link.pool->length;
}

/************************************************************
 *\brief set data to the given command message
 *\internal
 *\covers: DD_DRV_CMN_01_046
 *
 *\param[out] m		Pointer to the command message
 *\param[in]  id	Message ID
 *\param[in]  opcode	Message opcode
 *\param[in]  length	Message length
 *\param[in]  buf	Pointer to the buffer data
 *\param[in]  next	Pointer to the next message
 *\retval     m		Pointer to the command message
 ************************************************************/
static inline struct xf_message *
xf_create_msg(struct xf_message *m, u32 id, u32 opcode, u32 length, void *buf,
	      struct xf_message *next)
{
	if (m) {
		m->id = id;
		m->opcode = opcode;
		m->length = length;
		m->buffer = buf;
		m->next = next;
	}

	return m;
}

/*****************************************************************
 *\brief synchronous send and wait for response message from proxy
 *\internal
 *\covers: DD_DRV_CMN_01_038
 *
 *\param[in]	msg		Pointer to the command message
 *\retval	0		Success
 *\retval	-EINVAL		Failed
 ****************************************************************/
static int xf_send_and_receive(struct xf_message *msg)
{
	int err;
	int opcode = msg->opcode;

	/* reset the base flag */
	spin_lock(&base->lock);
	base->base_flag = FALSE;
	spin_unlock(&base->lock);

	err = base->cmd.send(base->client, (void *)msg);
	if (err != 0)
		return err;

	/* sleep and wait for the response wake up event */
	if (wait_event_interruptible(base->base_wait,
				     base->base_flag || base->err_flag)) {
		return -EINVAL;
	}

	if (base->err_flag != 0) {
		spin_lock(&base->lock);
		base->err_flag = FALSE;
		spin_unlock(&base->lock);
		return -EINVAL;
	}

	/* save the response message */
	/* PRQA S 3200 */
	memcpy(msg, &base->base_msg, sizeof(struct xf_message));

	/* check if the reponsed message is right */
	if (msg->opcode != opcode)
		return -EINVAL;

	return 0;
}

/*************************************************************
 *\brief send a message to proxy
 *\internal
 *\covers: DD_DRV_CMN_01_036
 *
 *\param[in]	msg		Pointer to the command message
 *\retval	0		Success
 *\retval	-EINVAL		Failed
 **************************************************************/
static inline int xf_send(struct xf_message *msg)
{
	return base->cmd.send(base->client, (void *)msg);
}

/** ***********************************************************
 *\brief receive message from proxy
 *\internal
 *\covers: DD_DRV_CMN_01_037
 *
 *\param[in]	msg		Pointer to store the response message
 *\retval	0		Success
 *\retval	-EINVAL		Failed
 **************************************************************/
static inline int xf_receive(struct xf_message *msg)
{
	/* do not wait here */
	return base->cmd.recv(base->client, (void *)msg);
}

/** ***********************************************************
 *\brief thread for receive response data
 *\internal
 *\covers: DD_DRV_CMN_01_039
 **************************************************************/
static int xf_response_thread(void *data)
{
	struct xf_message msg;
	struct xf_handle *handle;
	int id;

	/* set polling to wait the response message */
	spin_lock(&base->lock);
	base->wait_flag = FALSE;
	spin_unlock(&base->lock);

	while (!kthread_should_stop()) {
		if (base->cmd.poll(base->client, &base->wait_flag) != 0)
			continue;

		if (xf_receive(&msg) != 0)
			continue;

		/* get the handle id */
		id = XF_AP_CLIENT(msg.id);

		/* check the destination of the response message */
		if (id == 0) {
			/* message is from base control */
			/* PRQA S 3200 1*/
			memcpy(&base->base_msg, &msg,
			       sizeof(struct xf_message));

			spin_lock(&base->lock);
			base->base_flag = TRUE;
			spin_unlock(&base->lock);

			wake_up(&base->base_wait);
			continue;
		}

		/* get handle data */
		handle = xf_adsp_base_get_handle(id);

		if (handle == 0)
			continue;

		switch (msg.opcode) {
		case XF_EMPTY_THIS_BUFFER:	/* PRQA S 0594 */
			handle->cb->empty_buf_done(handle->private_data,
						   msg.opcode,
						   msg.length,
						   msg.buffer);
			break;
		case XF_FILL_THIS_BUFFER:	/* PRQA S 0594 */
			handle->cb->fill_buf_done(handle->private_data,
						  msg.opcode,
						  msg.length,
						  msg.buffer);
			break;
		case XF_SET_PARAM:		/* PRQA S 0594 4 */
		case XF_GET_PARAM:
		case XF_ROUTE:
		case XF_UNROUTE:
		case XF_MMAP_THIS_BUFFER:
			/* message is from base control */
			memcpy(&base->base_msg, &msg,
			       sizeof(struct xf_message)); /* PRQA S 3200 */

			spin_lock(&base->lock);
			base->base_flag = TRUE;
			spin_unlock(&base->lock);

			wake_up(&base->base_wait);
			break;
		default:
			/* error has occurred */
			handle->cb->event_handler(handle->private_data);

			xf_adsp_base_free_handle(id); /* PRQA S 3200 */

			spin_lock(&base->lock);
			base->err_flag = TRUE;
			spin_unlock(&base->lock);

			wake_up(&base->base_wait);
			break;
		}
	}

	pr_info("ADSP base thread was exited\n");

	do_exit(0);
	return 0;
}

/*************************************************************
 *	\brief register component to ADSP
 *	\internal
 *	\covers: DD_DRV_CMN_01_040
 *
 *	\param[in]    name      Name string of component
 *	\param[out]   comp_id   Store the registered component ID
 *	\retval	      0         Success
 *	\retval	      -EINVAL   Failed
 **************************************************************/
static int xf_adsp_register(char *name, int *comp_id)	/* PRQA S 3673 */
{
	struct xf_message msg;
	struct xf_buffer *b = xf_buffer_get(base->aux_pool);
	int err = 0;

	msg.id = __XF_MSG_ID(__XF_AP_PROXY(0), __XF_DSP_PROXY(0));
	msg.opcode = XF_REGISTER;
	msg.length = strlen(name) + 1;
	msg.buffer = xf_buffer_data(b);

	/* copy identify name of ADSP component */
	if (msg.length <= xf_buffer_length(b))
		strncpy(msg.buffer, name, msg.length);	/* PRQA S 3200 */
	else
		strncpy(msg.buffer, name, xf_buffer_length(b));/* PRQA S 3200 */

	err = xf_send_and_receive(&msg);
	if (err != 0)
		goto exit;		/* PRQA S 2001 */

	/* save the registered component ID */
	*comp_id = XF_MSG_SRC(msg.id);

exit:
	/* return msg to pool */
	xf_buffer_put(b);

	return err;
}

/** ***********************************************************
 * \brief unregister component from ADSP
 * \internal
 * \covers: DD_DRV_CMN_01_041
 * \param[in]	comp_id		Registered component ID
 * \retval	0		Success
 * \retval	-EINVAL		Failed
 **************************************************************/
static int xf_adsp_unregister(int comp_id)
{
	struct xf_message msg;

	xf_create_msg(&msg, __XF_MSG_ID(__XF_AP_PROXY(0), comp_id),
		      XF_UNREGISTER, 0, NULL, NULL);	/* PRQA S 3200 */

	return xf_send_and_receive(&msg);
}

/***************************************************************
 * APIs for ADSP component helper
 * ************************************************************/
/** ***********************************************************
 * \brief allocate memory pool from shared memory
 * \internal
 * \covers: DD_DRV_CMN_01_005
 * \param[in]   pool_size	Number of buffer need to allocate
 * \param[in]   buf_length	Size of each buffer in bytes
 * \retval	pool		Pointer to allocated pool
 * \retval	-EINVAL		Invalid base data
 * \retval	-ENOMEM		Out of memory resource
 **************************************************************/
struct xf_pool *xf_adsp_allocate_mem_pool(int pool_size, int buf_length)
{
	void *data;
	u32 number;
	struct xf_buffer *b;
	struct xf_message msg;
	struct xf_pool *pool;
	int err = 0;

	/* check the sane ADSP base data */
	if (!base)
		return ERR_PTR(-EINVAL);	/* PRQA S 0306 */

	xf_create_msg(&msg, __XF_MSG_ID(__XF_AP_PROXY(0), __XF_DSP_PROXY(0)),
		      XF_ALLOC,
		      buf_length * pool_size, NULL, NULL); /* PRQA S 3200 */

	err = xf_send_and_receive(&msg);
	if (err != 0 || !msg.buffer)
		return ERR_PTR(-ENOMEM);		/* PRQA S 0306 */

	/* PRQA S 0306 1 */
	pool = kmalloc(offset_of(struct xf_pool, buffer) +
		       (pool_size * sizeof(struct xf_buffer)), GFP_KERNEL);

	if (!pool)
		return ERR_PTR(-ENOMEM);	/* PRQA S 0306 */

	pool->length = buf_length;
	pool->number = pool_size;
	pool->p = msg.buffer;

	number = pool_size;
	for (pool->free = b = &pool->buffer[0], data = pool->p;
	     number > 0; number--, b++) {	/* PRQA S 2462,3418,0489 */
		/* set address of the buffer */
		b->address = data;

		/* fill buffer into free list */
		if (number == 1)
			b->link.next = NULL;
		else
			b->link.next = b + 1;		/* PRQA S 0489 */

		/* advance data pointer in contigous buffer */
		data += buf_length;		/* PRQA S 0550 */
	}

	return pool;
}

/** ***********************************************************
 *\brief return memory to shared memory
 *\internal
 *\covers: DD_DRV_CMN_01_006
 *\param[in]	pool	Data pool address
 *\retval	0	Success
 *\retval	-EINVAL Invalid base or pool data
 **************************************************************/
int xf_adsp_free_mem_pool(struct xf_pool *pool)
{
	struct xf_message msg;

	/* check the sane ADSP base pool data*/
	if (!base || !pool)
		return -EINVAL;

	xf_create_msg(&msg, __XF_MSG_ID(__XF_AP_PROXY(0), __XF_DSP_PROXY(0)),
		      XF_FREE, pool->number * pool->length,
		      pool->p, NULL);		/* PRQA S 3200 */

	xf_send_and_receive(&msg);		/* PRQA S 3200 */

	kfree(pool);

	return 0;
}

/** ***********************************************************
 *\brief get buffer from given pool
 *\internal
 *\covers: DD_DRV_CMN_01_007
 *
 *\param[in]	pool	Data pool address
 *\param[in]	index   Buffer index
 *\retval	b	Pointer to buffer address in pool
 *\retval	-EINVAL Invalid pool or index
 **************************************************************/
char *xf_adsp_get_data_from_pool(struct xf_pool *pool, int index)
{
	struct xf_buffer *buf[XF_BUF_POOL_SIZE] = {0};
	char *data;
	int i;

	/* check the sane pool data*/
	if (!pool)
		return ERR_PTR(-EINVAL);		/* PRQA S 306 */

	/* check the index is valid */
	if (index > (pool->number - 1))
		return ERR_PTR(-EINVAL);		/* PRQA S 306 */

	/* get buffer from pool */
	for (i = 0; i <= index; i++)
		buf[i] = xf_buffer_get(pool);

	/* get data from buffer */
	data = xf_buffer_data(buf[index]);

	/* return buffer to pool */
	for (i = index; i >= 0; i--)
		xf_buffer_put(buf[i]);

	return data;
}

/** ***********************************************************
 *\brief  send empty this buffer command to ADSP framework
 *\internal
 *\covers: DD_DRV_CMN_01_003
 *
 *\param[in]	handle_id	ID of the registered handle
 *\param[in]	buffer		Pointer to data buffer
 *\param[in]	length		Size of buffer in bytes
 *\retval	0		Success
 *\retval	-EINVAL		Failed
 **************************************************************/
int xf_adsp_empty_this_buffer(int handle_id, char *buffer, int length)
{
	struct xf_message msg;
	struct xf_handle *handle;

	/* check the sane ADSP base data */
	if (!base)
		return -EINVAL;

	handle = xf_adsp_base_get_handle(handle_id);
	if (!handle)
		return -EINVAL;

	/* submit message to port 0 of component */
	/* PRQA S 3200 2 */
	xf_create_msg(&msg,
		      __XF_MSG_ID(__XF_AP_CLIENT(0, handle_id),
				  __XF_PORT_SPEC2(handle->comp_id, 0)),
		      XF_EMPTY_THIS_BUFFER, length, buffer, NULL);

	xf_send(&msg);		/* PRQA S 3200 */

	return 0;
}

/** ***********************************************************
 *\brief  send fill this buffer command to ADSP framework
 *\internal
 *\covers: DD_DRV_CMN_01_004
 *
 *\param[in]	handle_id	ID of the registered handle
 *\param[in]	buffer		Pointer to data buffer
 *\param[in]	length		Size of buffer in bytes
 *\retval	0		Success
 *\retval	-EINVAL		Failed
 **************************************************************/
int xf_adsp_fill_this_buffer(int handle_id, char *buffer, int length)
{
	struct xf_message msg;
	struct xf_handle *handle;

	/* check the sane ADSP base data */
	if (!base)
		return -EINVAL;

	handle = xf_adsp_base_get_handle(handle_id);
	if (!handle)
		return -EINVAL;

	/* submit message to port 1 of component */
	/* PRQA S 3200 2 */
	xf_create_msg(&msg,
		      __XF_MSG_ID(__XF_AP_CLIENT(0, handle_id),
				  __XF_PORT_SPEC2(handle->comp_id, 1)),
		      XF_FILL_THIS_BUFFER, length, buffer, NULL);

	xf_send(&msg);		/* PRQA S 3200 */

	return 0;
}

/** ***********************************************************
 *\brief  send mmap this buffer command to ADSP framework
 *\internal
 *\covers: DD_DRV_CMN_01_052
 *
 *\param[in]	handle_id	ID of the registered handle
 *\param[in]	buffer		Pointer to data buffer
 *\param[in]	length		Size of buffer in bytes
 *\retval	0		Success
 *\retval	-EINVAL		Failed
 **************************************************************/
int xf_adsp_mmap_this_buffer(int handle_id, char *buffer, int length)
{
	struct xf_message msg;
	struct xf_handle *handle;

	/* check the sane ADSP base data */
	if (!base)
		return -EINVAL;

	handle = xf_adsp_base_get_handle(handle_id);
	if (!handle)
		return -EINVAL;

	/* submit message to component */
	xf_create_msg(&msg,
		      __XF_MSG_ID(__XF_AP_CLIENT(0, handle_id),
				  __XF_PORT_SPEC2(handle->comp_id, 0)),
		      XF_MMAP_THIS_BUFFER, length, buffer, NULL);

	/* wait for the successful from ADSP */
	return xf_send_and_receive(&msg);
}

/** ***********************************************************
 *\brief route data between two registered ADSP plugins
 *\internal
 *\covers: DD_DRV_CMN_01_010
 *
 *\param[in]	src_handle_id	Handle ID of source plugin
 *\param[in]	dst_handle_id	Handle ID of sink plugin
 *\param[in]	buf_cnt		Number of buffer in tunnel
 *\param[in]	buf_size	Size of each buffer in tunnel
 *\retval	0		Success
 *\retval	-EINVAL		Failed
 **************************************************************/
int
xf_adsp_route(int src_handle_id, int dst_handle_id, int buf_cnt, int buf_size)
{
	struct xf_route_port_msg *route_msg;
	struct xf_message msg;
	struct xf_buffer *b;
	struct xf_handle *dst_handle, *src_handle;
	int err = 0;

	/* check the sane ADSP base data */
	if (!base)
		return -EINVAL;

	/* get handle data */
	dst_handle = xf_adsp_base_get_handle(dst_handle_id);
	src_handle = xf_adsp_base_get_handle(src_handle_id);

	if (!dst_handle || !src_handle)
		return -EINVAL;

	b = xf_buffer_get(base->aux_pool);
	route_msg = xf_buffer_data(b);

	/* route information */
	route_msg->dst = __XF_PORT_SPEC2(dst_handle->comp_id, 0);
	route_msg->alloc_align = 4;
	route_msg->alloc_number = buf_cnt;
	route_msg->alloc_size = buf_size;

	/* PRQA S 3200 2*/
	xf_create_msg(&msg,
		      __XF_MSG_ID(__XF_AP_CLIENT(0, src_handle_id),
				  __XF_PORT_SPEC2(src_handle->comp_id, 1)),
		      XF_ROUTE, sizeof(*route_msg), route_msg, NULL);

	err = xf_send_and_receive(&msg);

	xf_buffer_put(b);

	return err;
}

/** ***********************************************************
 *\brief set a single parameter
 *\internal
 *\covers: DD_DRV_CMN_01_008
 *
 *\param[in]	handle_id	ID of registered handle
 *\param[in]	index		Sub-command index of parameter
 *\param[in]	value		the setting value
 *\retval	0		Success
 *\retval	-EINVAL		Failed
 **************************************************************/
int xf_adsp_set_param(int handle_id, int index, int value)
{
	struct xf_message msg;
	struct xf_buffer *b;
	struct xf_set_param_msg *msg_params;
	struct xf_handle *handle;
	int err = 0;

	/* check the sane ADSP base data */
	if (!base)
		return -EINVAL;

	handle = xf_adsp_base_get_handle(handle_id);
	if (!handle)
		return -EINVAL;

	b = xf_buffer_get(base->aux_pool);
	msg_params = xf_buffer_data(b);

	msg_params->item[0].id = index;
	msg_params->item[0].value = value;

	/* PRQA S 3200 2 */
	xf_create_msg(&msg,
		      __XF_MSG_ID(__XF_AP_CLIENT(0, handle_id),
				  __XF_PORT_SPEC2(handle->comp_id, 0)),
		      XF_SET_PARAM, XF_SET_PARAM_CMD_LEN(1), msg_params, NULL);

	err = xf_send_and_receive(&msg);

	/* return msg to pool */
	xf_buffer_put(b);

	return err;
}

/** ***********************************************************
 *\brief get a single parameter
 *\internal
 *\covers: DD_DRV_CMN_01_009
 *
 *\param[in]	handle_id	ID of registered handle
 *\param[in]	index		Sub-command index of parameter
 *\param[out]	value		Store the getting value
 *\retval	0		Success
 *\retval	-EINVAL		Failed
 **************************************************************/
int xf_adsp_get_param(int handle_id, int index, int *value)
{
	struct xf_message msg;
	struct xf_buffer *b;
	union xf_get_param_msg *msg_params;
	struct xf_handle *handle;
	int err = 0;

	/* check the sane ADSP base and value data */
	if (!base || !value)
		return -EINVAL;

	handle = xf_adsp_base_get_handle(handle_id);
	if (!handle)
		return -EINVAL;

	b = xf_buffer_get(base->aux_pool);
	msg_params = xf_buffer_data(b);

	msg_params->c.id[0] = index;

	/* PRQA S 3200 2 */
	xf_create_msg(&msg,
		      __XF_MSG_ID(__XF_AP_CLIENT(0, handle_id),
				  __XF_PORT_SPEC2(handle->comp_id, 0)),
		      XF_GET_PARAM, XF_GET_PARAM_CMD_LEN(1), msg_params, NULL);

	err = xf_send_and_receive(&msg);
	if (err != 0)
		goto exit;		/* PRQA S 2001 */

	/* save the received parameters */
	*value = msg_params->r.value[0];

exit:
	/* return msg to pool */
	xf_buffer_put(b);

	return err;
}

/***************************************************
 * APIs for ADSP base control
 * ************************************************/

/** ***********************************************************
 *\brief initialize ADSP base's instance
 *\internal
 *\covers: DD_DRV_CMN_01_001
 *
 *\retval 0	  Success
 *\retval -EINVAL ADSP base's instance has been initialized
 *\retval -ENOMEM Cannot allocate memory for ADSP base
 *\retval -EBUSY  Cannot register client to proxy
 **************************************************************/
int xf_adsp_base_create(struct xf_adsp_base_cmd *cmd)
{
	int err = 0;

	if (base != 0)
		return -EINVAL;

	if (!cmd || !cmd->recv || !cmd->send || !cmd->poll ||
	    !cmd->client_register || !cmd->client_unregister)
		return -EINVAL;

	base = kmalloc(sizeof(*base), GFP_KERNEL);
	if (!base)
		return -ENOMEM;

	memset(base, 0, sizeof(struct xf_adsp_base));	/* PRQA S 3200 */

	/* store the proxy command */
	memcpy(&base->cmd, cmd, sizeof(struct xf_adsp_base_cmd));

	/* create client to connect from proxy driver */
	err = base->cmd.client_register(&base->client);
	if (err != 0)
		goto err3;		/* PRQA S 2001 */

	/* initialize waiting queue */
	init_waitqueue_head(&base->base_wait);

	/* initialize handle */
	xf_adsp_base_init_handle();

	/* create thread to get the responsed message from proxy */
	base->rsp_thread = kthread_run(&xf_response_thread,
				       (void *)base, "adsp base");

	if (base->rsp_thread != 0) {
		pr_info("ADSP base thread has been started.\n");
	} else {
		pr_info("Failed in create base thread\n");
		err = -ENOMEM;
		goto err2;		/* PRQA S 2001 */
	}

	/* allocate auxiliary pool for component usage */
	base->aux_pool = xf_adsp_allocate_mem_pool(
					XF_AUX_POOL_SIZE,
					XF_ALIGNED(XF_AUX_POOL_MSG_LENGTH));

	if (IS_ERR(base->aux_pool)) {		/* PRQA S 306 */
		err = -ENOMEM;
		goto err1;		/* PRQA S 2001 */
	}

	pr_info("ADSP base was created\n");
	return 0;

err1:
	/* cancel the waitting queue */
	spin_lock(&base->lock);
	base->wait_flag = TRUE;
	spin_unlock(&base->lock);

	/* stop thread inadvance */
	kthread_stop(base->rsp_thread);

err2:
	base->cmd.client_unregister(base->client);

err3:
	kfree(base);
	base = NULL;

	return err;
}
EXPORT_SYMBOL(xf_adsp_base_create);		/* PRQA S 0651 */

/** ***********************************************************
 *\brief deinitialize ADSP base's instance
 *\internal
 *\covers: DD_DRV_CMN_01_002
 *
 *\retval	 0	   Success
 *\retval	 -EINVAL Invalid ADSP base's instance
 **************************************************************/
int xf_adsp_base_destroy(void)
{
	/* check the sane ADSP base data */
	if (!base)
		return -EINVAL;

	/* free auxiliary pool to shared memory */
	xf_adsp_free_mem_pool(base->aux_pool);		/* PRQA S 3200 */

	/* cancel wait the response message, go to stop process */
	spin_lock(&base->lock);
	base->wait_flag = TRUE;
	spin_unlock(&base->lock);

	/* stop response thread */
	kthread_stop(base->rsp_thread);

	/* unregister client */
	base->cmd.client_unregister(base->client);

	kfree(base);
	base = NULL;

	pr_info("ADSP base was destroyed\n");
	return 0;
}
EXPORT_SYMBOL(xf_adsp_base_destroy);		/* PRQA S 0651 */

/** ***********************************************************
 *	\brief initialize handle instance
 *	\internal
 *	\covers: DD_DRV_CMN_01_031
 **************************************************************/
static inline void xf_adsp_base_init_handle(void)
{
	int i;

	for (i = 0; i < MAX_HANDLE; i++)
		base->handle[i] = NULL;
}

/** ***********************************************************
 *\brief get the next available handle ID for register
 *\internal
 *\covers: DD_DRV_CMN_01_032
 *
 *\retval -1		Unavailable handle ID
 *\retval 0 to 255	Available handle ID
 **************************************************************/
static inline int xf_adsp_base_get_valid_handle(void)
{
	int id = -1;
	int i;

	for (i = 0; i < MAX_HANDLE; i++) {
		/* get the id of the first available handler */
		if (!base->handle[i]) {
			id = i;
			break;
		}
	}

	return id;
}

/** ***********************************************************
 *\brief register a handle instance for component usage
 *\internal
 *\covers: DD_DRV_CMN_01_033
 *
 *\param[in]	 private_data	Private data of this component
 *\param[in]	 cb		Callback function
 *\param[in]	 comp_id	ID of register component
 *\retval	 id		ID of registered handle
 *\retval	 -EINVAL	Cannot get the handle instance
 *\retval	 -ENOMEM	Cannot allocate handle memory
 **************************************************************/
static int xf_adsp_base_register_handle(void *private_data,
					struct xf_callback_func *cb,
					int comp_id)
{
	struct xf_handle *handle;
	int id;

	/* get the next handle id */
	id = xf_adsp_base_get_valid_handle();

	/* check available handle in base */
	if (id < 0)
		return -EINVAL;

	/* allocate handle data */
	handle = kmalloc(sizeof(*handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	/* save handle data */
	handle->private_data = private_data;
	handle->cb = cb;
	handle->comp_id = comp_id;

	/* save the hanle data */
	base->handle[id] = handle;

	/* return the id numbering in base handle */
	return (id + 1);
}

/** ***********************************************************
 *\brief  get handle instance from handle ID
 *\internal
 *\covers: DD_DRV_CMN_01_034
 *
 *\param[in]	 handle_id	ID of registered handle
 *\retval	 handle		Pointer to handle instance
 **************************************************************/
static inline struct xf_handle *xf_adsp_base_get_handle(int handle_id)
{
	return base->handle[handle_id - 1];
}

/** ***********************************************************
 *\brief free the registered handle instance
 *\internal
 *\covers: DD_DRV_CMN_01_035
 *
 *\param[in]	handle_id	ID of registered handle
 *\retval	0		Success
 *\retval	-EINVAL		Invalid handle ID
 **************************************************************/
static int xf_adsp_base_free_handle(int handle_id)
{
	if (handle_id < 1 || handle_id > MAX_HANDLE)
		return -EINVAL;

	kfree(base->handle[handle_id - 1]);
	base->handle[handle_id - 1] = NULL;

	return 0;
}

/***********************************************************************
 * APIs for Renderer component
 * ********************************************************************/

/** ***********************************************************
 *\brief  set ADSP Renderer parameters
 *\internal
 *\covers: DD_DRV_CMN_01_013
 *
 *\param[in]	renderer	Pointer to Renderer component
 *\retval	0		Success
 *\retval	-EINVAL		Failed
 **************************************************************/
int xf_adsp_renderer_set_params(struct xf_adsp_renderer *renderer)
{
	struct xf_message msg;
	struct xf_set_param_msg *msg_params;
	struct xf_adsp_renderer_params *params;
	struct xf_buffer *b;
	int i;
	struct xf_handle *handle;
	int err = 0;

	/* check the sane ADSP base data */
	if (!base || !renderer)
		return -EINVAL;

	params = &renderer->params;

	/* get Renderer's handle data */
	handle = xf_adsp_base_get_handle(renderer->handle_id);
	if (!handle)
		return -EINVAL;

	b = xf_buffer_get(base->aux_pool);

	msg_params = xf_buffer_data(b);

	i = 0;
	msg_params->item[i].id = XA_RDR_CONFIG_PARAM_SAMPLE_RATE;
	msg_params->item[i++].value = params->in_rate;

	msg_params->item[i].id = XA_RDR_CONFIG_PARAM_CHANNELS;
	msg_params->item[i++].value = params->channel;

	msg_params->item[i].id = XA_RDR_CONFIG_PARAM_PCM_WIDTH;
	msg_params->item[i++].value = params->pcm_width;

	msg_params->item[i].id = XA_RDR_CONFIG_PARAM_FRAME_SIZE;
	msg_params->item[i++].value = params->frame_size;

	msg_params->item[i].id = XA_RDR_CONFIG_PARAM_OUTPUT1;
	msg_params->item[i++].value = params->dev1;

	msg_params->item[i].id = XA_RDR_CONFIG_PARAM_OUTPUT2;
	msg_params->item[i++].value = params->dev2;

	msg_params->item[i].id = XA_RDR_CONFIG_PARAM_DMACHANNEL1;
	msg_params->item[i++].value = params->dma1;

	msg_params->item[i].id = XA_RDR_CONFIG_PARAM_DMACHANNEL2;
	msg_params->item[i++].value = params->dma2;

	msg_params->item[i].id = XA_RDR_CONFIG_PARAM_OUT_SAMPLE_RATE;
	msg_params->item[i++].value = params->out_rate;

	msg_params->item[i].id = XA_RDR_CONFIG_PARAM_VOLUME_RATE;
	msg_params->item[i++].value = params->vol_rate;

	msg_params->item[i].id = XA_RDR_CONFIG_PARAM_OUT_CHANNELS;
	msg_params->item[i++].value = params->out_channel;

	msg_params->item[i].id = XA_RDR_CONFIG_PARAM_MIX_CONTROL;
	msg_params->item[i++].value = params->mix_ctrl;

	msg_params->item[i].id = XA_RDR_CONFIG_PARAM_STATE;
	msg_params->item[i++].value = params->state;

	msg_params->item[i].id = XA_RDR_CONFIG_PARAM_RING_NUM;
	msg_params->item[i++].value = params->ring_num;

	/* PRQA S 3200 2*/
	xf_create_msg(&msg,
		      __XF_MSG_ID(__XF_AP_CLIENT(0, renderer->handle_id),
				  __XF_PORT_SPEC2(handle->comp_id, 0)),
		      XF_SET_PARAM, XF_SET_PARAM_CMD_LEN(i), msg_params, NULL);

	err = xf_send_and_receive(&msg);

	/* return msg to pool */
	xf_buffer_put(b);

	return err;
}

/** ***********************************************************
 *\brief  get ADSP Renderer parameters
 *\internal
 *\covers: DD_DRV_CMN_01_014
 *
 *\param[in]	renderer	Pointer to Renderer component
 *\retval	0		Success
 *\retval	-EINVAL		Failed
 **************************************************************/
int xf_adsp_renderer_get_params(struct xf_adsp_renderer *renderer)
{
	struct xf_adsp_renderer_params *rdr_params;
	struct xf_message msg;
	struct xf_buffer *b;
	union xf_get_param_msg *msg_params;
	int i;
	struct xf_handle *handle;
	int err = 0;

	/* check the sane ADSP base data */
	if (!base || !renderer)
		return -EINVAL;

	rdr_params = &renderer->params;

	/* get Renderer's handle data */
	handle = xf_adsp_base_get_handle(renderer->handle_id);
	if (!handle)
		return -EINVAL;

	b = xf_buffer_get(base->aux_pool);
	msg_params = xf_buffer_data(b);

	i = 0;
	/* PRQA S 3440 13 1*/
	msg_params->c.id[i++] = XA_RDR_CONFIG_PARAM_SAMPLE_RATE;
	msg_params->c.id[i++] = XA_RDR_CONFIG_PARAM_CHANNELS;
	msg_params->c.id[i++] = XA_RDR_CONFIG_PARAM_PCM_WIDTH;
	msg_params->c.id[i++] = XA_RDR_CONFIG_PARAM_FRAME_SIZE;
	msg_params->c.id[i++] = XA_RDR_CONFIG_PARAM_OUTPUT1;
	msg_params->c.id[i++] = XA_RDR_CONFIG_PARAM_OUTPUT2;
	msg_params->c.id[i++] = XA_RDR_CONFIG_PARAM_DMACHANNEL1;
	msg_params->c.id[i++] = XA_RDR_CONFIG_PARAM_DMACHANNEL2;
	msg_params->c.id[i++] = XA_RDR_CONFIG_PARAM_OUT_SAMPLE_RATE;
	msg_params->c.id[i++] = XA_RDR_CONFIG_PARAM_VOLUME_RATE;
	msg_params->c.id[i++] = XA_RDR_CONFIG_PARAM_OUT_CHANNELS;
	msg_params->c.id[i++] = XA_RDR_CONFIG_PARAM_MIX_CONTROL;
	msg_params->c.id[i++] = XA_RDR_CONFIG_PARAM_STATE;
	msg_params->c.id[i++] = XA_RDR_CONFIG_PARAM_RING_NUM;

	/* PRQA S 3200 2*/
	xf_create_msg(&msg,
		      __XF_MSG_ID(__XF_AP_CLIENT(0, renderer->handle_id),
				  __XF_PORT_SPEC2(handle->comp_id, 0)),
		      XF_GET_PARAM, XF_GET_PARAM_CMD_LEN(i), msg_params, NULL);

	err = xf_send_and_receive(&msg);
	if (err != 0)
		goto exit;		/* PRQA S 2001 */

	/* save the received parameters */
	i = 0;
	rdr_params->in_rate = msg_params->r.value[i++];	/* PRQA S 3440 13 */
	rdr_params->channel = msg_params->r.value[i++];
	rdr_params->pcm_width = msg_params->r.value[i++];
	rdr_params->frame_size = msg_params->r.value[i++];
	rdr_params->dev1 = msg_params->r.value[i++];
	rdr_params->dev2 = msg_params->r.value[i++];
	rdr_params->dma1 = msg_params->r.value[i++];
	rdr_params->dma2 = msg_params->r.value[i++];
	rdr_params->out_rate = msg_params->r.value[i++];
	rdr_params->vol_rate = msg_params->r.value[i++];
	rdr_params->out_channel = msg_params->r.value[i++];
	rdr_params->mix_ctrl = msg_params->r.value[i++];
	rdr_params->state = msg_params->r.value[i++];
	rdr_params->ring_num = msg_params->r.value[i++];

exit:
	/* return msg to pool */
	xf_buffer_put(b);

	return err;
}

/** **************************************************************************
 *\brief  create Renderer component
 *\internal
 *\covers: DD_DRV_CMN_01_011
 *
 *\param[in,out]  renderer	Pointer to the registered component
 *\param[in]	  cb		Callback function
 *\param[in]	  private_data	Private data
 *\retval	  0		Success
 *\retval	 -EINVAL	Invalid base instance or register fail
 *\retval	 -ENOMEM	Cannot allocate Renderer instance
 *****************************************************************************/
int xf_adsp_renderer_create(struct xf_adsp_renderer **renderer,
			    struct xf_callback_func *cb, void *private_data)
{
	struct xf_adsp_renderer *rdr;
	int err;
	int comp_id;

	/* check the sane ADSP base data */
	if (!base)
		return -EINVAL;

	rdr = kmalloc(sizeof(*rdr), GFP_KERNEL);
	if (!rdr)
		return -ENOMEM;

	memset(rdr, 0, sizeof(struct xf_adsp_renderer)); /* PRQA S 3200 */

	/* register renderer component */
	err = xf_adsp_register("renderer", &comp_id);
	if (err != 0)
		goto err2;		/* PRQA S 2001 */

	/* register Renderer to ADSP base control */
	rdr->handle_id = xf_adsp_base_register_handle(private_data,
						      cb, comp_id);

	if (rdr->handle_id <= 0) {
		err = -EINVAL;
		goto err1;		/* PRQA S 2001 */
	}

	/* get the default parameter from Renderer plugin */
	err = xf_adsp_renderer_get_params(rdr);
	if (err != 0)
		goto err1;		/* PRQA S 2001 */

	/* save renderer compoent data */
	*renderer = rdr;

	return 0;

err1:
	xf_adsp_unregister(comp_id);		/* PRQA S 3200 */

err2:
	kfree(rdr);

	return err;
}

/** ***********************************************************
 *\brief  deinitialize ADSP Renderer component
 *\internal
 *\covers: DD_DRV_CMN_01_012
 *
 *\param[in]	 renderer	Pointer to Renderer component
 *\retval	 0		Success
 *\retval	 -EINVAL	Invalid base or Renderer data
 **************************************************************/
int xf_adsp_renderer_destroy(struct xf_adsp_renderer *renderer)
{
	struct xf_handle *handle;
	int handle_id;

	/* check the sane ADSP base data */
	if (!base || !renderer)
		return -EINVAL;

	handle_id = renderer->handle_id;

	handle = xf_adsp_base_get_handle(handle_id);
	if (!handle)
		goto exit;		/* PRQA S 2001 */

	/* unregister component */
	xf_adsp_unregister(handle->comp_id);		/* PRQA S 3200 */

	/* free handle data from base control */
	xf_adsp_base_free_handle(handle_id);		/* PRQA S 3200 */

exit:
	kfree(renderer);

	return 0;
}

/***********************************************************************
 * APIs for Capture component
 * ********************************************************************/

/** ***********************************************************
 *\brief  set ADSP Capture parameters
 *\internal
 *\covers: DD_DRV_CMN_01_017
 *
 *\param[in]	 capture	 Pointer to Capture component
 *\retval	 0		 Success
 *\retval	 -EINVAL	 Failed
 **************************************************************/
int xf_adsp_capture_set_params(struct xf_adsp_capture *capture)
{
	struct xf_message msg;
	struct xf_set_param_msg *msg_params;
	struct xf_adsp_capture_params *params;
	struct xf_buffer *b;
	int i;
	struct xf_handle *handle;
	int err = 0;

	/* check the sane ADSP base data */
	if (!base || !capture)
		return -EINVAL;

	params = &capture->params;

	/* get Capture's handle data */
	handle = xf_adsp_base_get_handle(capture->handle_id);
	if (!handle)
		return -EINVAL;

	b = xf_buffer_get(base->aux_pool);

	msg_params = xf_buffer_data(b);

	i = 0;
	msg_params->item[i].id = XA_CAP_CONFIG_PARAM_SAMPLE_RATE;
	msg_params->item[i++].value = params->in_rate;

	msg_params->item[i].id = XA_CAP_CONFIG_PARAM_CHANNELS;
	msg_params->item[i++].value = params->channel;

	msg_params->item[i].id = XA_CAP_CONFIG_PARAM_PCM_WIDTH;
	msg_params->item[i++].value = params->pcm_width;

	msg_params->item[i].id = XA_CAP_CONFIG_PARAM_FRAME_SIZE;
	msg_params->item[i++].value = params->frame_size;

	msg_params->item[i].id = XA_CAP_CONFIG_PARAM_INPUT1;
	msg_params->item[i++].value = params->dev1;

	msg_params->item[i].id = XA_CAP_CONFIG_PARAM_INPUT2;
	msg_params->item[i++].value = params->dev2;

	msg_params->item[i].id = XA_CAP_CONFIG_PARAM_DMACHANNEL1;
	msg_params->item[i++].value = params->dma1;

	msg_params->item[i].id = XA_CAP_CONFIG_PARAM_DMACHANNEL2;
	msg_params->item[i++].value = params->dma2;

	msg_params->item[i].id = XA_CAP_CONFIG_PARAM_OUT_SAMPLE_RATE;
	msg_params->item[i++].value = params->out_rate;

	msg_params->item[i].id = XA_CAP_CONFIG_PARAM_VOLUME_RATE;
	msg_params->item[i++].value = params->vol_rate;

	msg_params->item[i].id = XA_CAP_CONFIG_PARAM_STATE;
	msg_params->item[i++].value = params->state;

	msg_params->item[i].id = XA_CAP_CONFIG_PARAM_RING_NUM;
	msg_params->item[i++].value = params->ring_num;

	/* PRQA S 3200 2 */
	xf_create_msg(&msg,
		      __XF_MSG_ID(__XF_AP_CLIENT(0, capture->handle_id),
				  __XF_PORT_SPEC2(handle->comp_id, 0)),
		      XF_SET_PARAM, XF_SET_PARAM_CMD_LEN(i), msg_params, NULL);

	err = xf_send_and_receive(&msg);

	/* return msg to pool */
	xf_buffer_put(b);

	return err;
}

/** ***********************************************************
 *\brief  get ADSP Capture parameters
 *\internal
 *\covers: DD_DRV_CMN_01_018
 *
 *\param[in]	 capture	 Pointer to Capture component
 *\retval	 0		 Success
 *\retval	 -EINVAL	 Failed
 **************************************************************/
int xf_adsp_capture_get_params(struct xf_adsp_capture *capture)
{
	struct xf_adsp_capture_params *cap_params;
	struct xf_message msg;
	struct xf_buffer *b;
	union xf_get_param_msg *msg_params;
	int i;
	struct xf_handle *handle;
	int err = 0;

	/* check the sane ADSP base data */
	if (!base || !capture)
		return -EINVAL;

	cap_params = &capture->params;

	/* get Capture's handle data */
	handle = xf_adsp_base_get_handle(capture->handle_id);
	if (!handle)
		return -EINVAL;

	b = xf_buffer_get(base->aux_pool);
	msg_params = xf_buffer_data(b);

	i = 0;
	/* PRQA S 3440 11 1*/
	msg_params->c.id[i++] = XA_CAP_CONFIG_PARAM_SAMPLE_RATE;
	msg_params->c.id[i++] = XA_CAP_CONFIG_PARAM_CHANNELS;
	msg_params->c.id[i++] = XA_CAP_CONFIG_PARAM_PCM_WIDTH;
	msg_params->c.id[i++] = XA_CAP_CONFIG_PARAM_FRAME_SIZE;
	msg_params->c.id[i++] = XA_CAP_CONFIG_PARAM_INPUT1;
	msg_params->c.id[i++] = XA_CAP_CONFIG_PARAM_INPUT2;
	msg_params->c.id[i++] = XA_CAP_CONFIG_PARAM_DMACHANNEL1;
	msg_params->c.id[i++] = XA_CAP_CONFIG_PARAM_DMACHANNEL2;
	msg_params->c.id[i++] = XA_CAP_CONFIG_PARAM_OUT_SAMPLE_RATE;
	msg_params->c.id[i++] = XA_CAP_CONFIG_PARAM_VOLUME_RATE;
	msg_params->c.id[i++] = XA_CAP_CONFIG_PARAM_STATE;
	msg_params->c.id[i++] = XA_CAP_CONFIG_PARAM_RING_NUM;

	/* PRQA S 3200 2 */
	xf_create_msg(&msg,
		      __XF_MSG_ID(__XF_AP_CLIENT(0, capture->handle_id),
				  __XF_PORT_SPEC2(handle->comp_id, 0)),
		      XF_GET_PARAM, XF_GET_PARAM_CMD_LEN(i), msg_params, NULL);

	err = xf_send_and_receive(&msg);
	if (err != 0)
		goto exit;		/* PRQA S 2001 */

	/* save the received parameters */
	i = 0;
	cap_params->in_rate = msg_params->r.value[i++];	/* PRQA S 3440 11 */
	cap_params->channel = msg_params->r.value[i++];
	cap_params->pcm_width = msg_params->r.value[i++];
	cap_params->frame_size = msg_params->r.value[i++];
	cap_params->dev1 = msg_params->r.value[i++];
	cap_params->dev2 = msg_params->r.value[i++];
	cap_params->dma1 = msg_params->r.value[i++];
	cap_params->dma2 = msg_params->r.value[i++];
	cap_params->out_rate = msg_params->r.value[i++];
	cap_params->vol_rate = msg_params->r.value[i++];
	cap_params->state = msg_params->r.value[i++];
	cap_params->ring_num = msg_params->r.value[i++];

exit:
	/* return msg to pool */
	xf_buffer_put(b);

	return err;
}

/** **************************************************************************
 *\brief  create Capture component
 *\internal
 *\covers: DD_DRV_CMN_01_015
 *
 *\param[in,out]  capture	 Pointer to the registered component
 *\param[in]	  cb		 Callback function
 *\param[in]	  private_data	 Private data
 *\retval	  0		 Success
 *\retval	  -EINVAL	 Invalid base instance or register fail
 *\retval	  -ENOMEM	 Cannot allocate Capture instance
 *****************************************************************************/
int xf_adsp_capture_create(struct xf_adsp_capture **capture,
			   struct xf_callback_func *cb, void *private_data)
{
	struct xf_adsp_capture *cap;
	int err;
	int comp_id;

	/* check the sane ADSP base data */
	if (!base)
		return -EINVAL;

	cap = kmalloc(sizeof(*cap), GFP_KERNEL);
	if (!cap)
		return -ENOMEM;

	memset(cap, 0, sizeof(struct xf_adsp_capture));	/* PRQA S 3200 */

	/* register capture component */
	err = xf_adsp_register("capture", &comp_id);
	if (err != 0)
		goto err2;		/* PRQA S 2001 */

	/* register capture to ADSP base control */
	cap->handle_id = xf_adsp_base_register_handle(private_data,
						      cb, comp_id);

	if (cap->handle_id <= 0) {
		err = -EINVAL;
		goto err1;		/* PRQA S 2001 */
	}

	/* get the default parameter from capture plugin */
	err = xf_adsp_capture_get_params(cap);
	if (err != 0)
		goto err1;		/* PRQA S 2001 */

	/* save capture compoent data */
	*capture = cap;

	return 0;

err1:
	xf_adsp_unregister(comp_id);	/* PRQA S 3200 */

err2:
	kfree(cap);

	return err;
}

/** ***********************************************************
 *\brief  deinitialize ADSP Capture component
 *\internal
 *\covers: DD_DRV_CMN_01_016
 *
 *\param[in]	 capture	 Pointer to Capture component
 *\retval	 0		 Success
 *\retval	 -EINVAL	 Invalid base or Capture data
 **************************************************************/
int xf_adsp_capture_destroy(struct xf_adsp_capture *capture) /* PRQA S 3673 */
{
	struct xf_handle *handle;
	int handle_id;

	/* check the sane ADSP base data */
	if (!base || !capture)
		return -EINVAL;

	handle_id = capture->handle_id;

	handle = xf_adsp_base_get_handle(handle_id);
	if (!handle)
		goto exit;		/* PRQA S 2001 */

	/* unregister component */
	xf_adsp_unregister(handle->comp_id);	/* PRQA S 3200 */

	/* free handle data from base control */
	xf_adsp_base_free_handle(handle_id);	/* PRQA S 3200 */

exit:
	kfree(capture);

	return 0;
}

/***********************************************************************
 * APIs for Equalizer component
 * ********************************************************************/

/** ***********************************************************
 *\brief  set ADSP Equalizer parameters
 *\internal
 *\covers: DD_DRV_CMN_01_021
 *
 *\param[in]	 equalizer	 Pointer to Equalizer component
 *\retval	 0		 Success
 *\retval	 -EINVAL	 Failed
 **************************************************************/
int xf_adsp_equalizer_set_params(struct xf_adsp_equalizer *equalizer)
{
	struct xf_message msg;
	struct xf_set_param_msg *msg_params;
	struct xf_adsp_equalizer_params    *params;
	struct xf_buffer    *b;
	int i, n;
	struct xf_handle *handle;
	int err = 0;

	/* check the sane ADSP base data */
	if (!base || !equalizer)
		return -EINVAL;

	params = &equalizer->params;

	/* get Equalizer's handle data */
	handle = xf_adsp_base_get_handle(equalizer->handle_id);
	if (!handle)
		return -EINVAL;

	b = xf_buffer_get(base->aux_pool);

	msg_params = xf_buffer_data(b);

	i = 0;
	msg_params->item[i].id = XA_EQZ_CONFIG_PARAM_COEF_FS;
	msg_params->item[i++].value = params->rate;

	msg_params->item[i].id = XA_EQZ_CONFIG_PARAM_CH;
	msg_params->item[i++].value = params->channel;

	msg_params->item[i].id = XA_EQZ_CONFIG_PARAM_PCM_WIDTH;
	msg_params->item[i++].value = params->pcm_width;

	msg_params->item[i].id = XA_EQZ_CONFIG_PARAM_EQZ_TYPE;
	msg_params->item[i++].value = params->eqz_type;

	if (params->eqz_type == XA_REL_EQZ_TYPE_PARAMETRIC) {
		struct xf_equalizer_parametric_coef *coef = &params->p_coef;

		for (n = 0; n < XA_REL_EQZ_FILTER_NUM; n++) {
			msg_params->item[i].id =
				XA_EQZ_CONFIG_PARAM_FILTER_0_COEF_TYPE + n;
			msg_params->item[i++].value = coef->type[n];

			msg_params->item[i].id =
				XA_EQZ_CONFIG_PARAM_FILTER_0_COEF_FC + n;
			msg_params->item[i++].value = coef->fc[n];

			msg_params->item[i].id =
				XA_EQZ_CONFIG_PARAM_FILTER_0_COEF_BW + n;
			msg_params->item[i++].value = coef->band_width[n];

			msg_params->item[i].id =
				XA_EQZ_CONFIG_PARAM_FILTER_0_COEF_GA + n;
			msg_params->item[i++].value = coef->gain[n];

			msg_params->item[i].id =
				XA_EQZ_CONFIG_PARAM_FILTER_0_COEF_BA + n;
			msg_params->item[i++].value = coef->gain_base[n];
		}
	} else {
		struct xf_equalizer_graphic_coef *coef = &params->g_coef;

		for (n = 0; n < XA_REL_EQZ_GRAPHIC_BAND_NUM; n++) {
			msg_params->item[i].id =
				XA_EQZ_CONFIG_PARAM_BAND_0_GCOEF_GA + n;
			msg_params->item[i++].value = coef->gain_g[n];
		}
	}

	/* PRQA S 3200 2 */
	xf_create_msg(&msg,
		      __XF_MSG_ID(__XF_AP_CLIENT(0, equalizer->handle_id),
				  __XF_PORT_SPEC2(handle->comp_id, 0)),
		      XF_SET_PARAM, XF_SET_PARAM_CMD_LEN(i), msg_params, NULL);

	err = xf_send_and_receive(&msg);

	/* return msg to pool */
	xf_buffer_put(b);

	return err;
}

/** ***********************************************************
 *\brief  get ADSP Equalizer parameters
 *\internal
 *\covers: DD_DRV_CMN_01_022
 *
 *\param[in]	 equalizer	 Pointer to Equalizer component
 *\retval	 0		 Success
 *\retval	 -EINVAL	 Failed
 **************************************************************/
int xf_adsp_equalizer_get_params(struct xf_adsp_equalizer *equalizer)
{
	struct xf_adsp_equalizer_params *eqz_params;
	struct xf_message msg;
	struct xf_buffer *b;
	union xf_get_param_msg *msg_params;
	int i, n;
	struct xf_handle *handle;
	int err = 0;

	/* check the sane ADSP base data */
	if (!base || !equalizer)
		return -EINVAL;

	eqz_params = &equalizer->params;

	/* get Equalizer's handle data */
	handle = xf_adsp_base_get_handle(equalizer->handle_id);
	if (!handle)
		return -EINVAL;

	b = xf_buffer_get(base->aux_pool);
	msg_params = xf_buffer_data(b);

	i = 0;
	msg_params->c.id[i++] = XA_EQZ_CONFIG_PARAM_COEF_FS; /* PRQA S 3440 4 */
	msg_params->c.id[i++] = XA_EQZ_CONFIG_PARAM_CH;
	msg_params->c.id[i++] = XA_EQZ_CONFIG_PARAM_PCM_WIDTH;
	msg_params->c.id[i++] = XA_EQZ_CONFIG_PARAM_EQZ_TYPE;

	for (n = 0; n < XA_REL_EQZ_FILTER_NUM; n++) {
		/* PRQA S 3440 5 */
		msg_params->c.id[i++] =
			XA_EQZ_CONFIG_PARAM_FILTER_0_COEF_TYPE + n;

		msg_params->c.id[i++] =
			XA_EQZ_CONFIG_PARAM_FILTER_0_COEF_FC + n;

		msg_params->c.id[i++] =
			XA_EQZ_CONFIG_PARAM_FILTER_0_COEF_BW + n;

		msg_params->c.id[i++] =
			XA_EQZ_CONFIG_PARAM_FILTER_0_COEF_GA + n;

		msg_params->c.id[i++] =
			XA_EQZ_CONFIG_PARAM_FILTER_0_COEF_BA + n;
	}

	for (n = 0; n < XA_REL_EQZ_GRAPHIC_BAND_NUM; n++)
		/* PRQA S 3440 1 */
		msg_params->c.id[i++] = XA_EQZ_CONFIG_PARAM_BAND_0_GCOEF_GA + n;

	/* PRQA S 3200 2 */
	xf_create_msg(&msg,
		      __XF_MSG_ID(__XF_AP_CLIENT(0, equalizer->handle_id),
				  __XF_PORT_SPEC2(handle->comp_id, 0)),
		      XF_GET_PARAM, XF_GET_PARAM_CMD_LEN(i), msg_params, NULL);

	err = xf_send_and_receive(&msg);
	if (err != 0)
		goto exit;		/* PRQA S 2001 */

	/* save the received parameters */
	i = 0;
	eqz_params->rate = msg_params->r.value[i++];	/* PRQA S 3440 4 */
	eqz_params->channel = msg_params->r.value[i++];
	eqz_params->pcm_width = msg_params->r.value[i++];
	eqz_params->eqz_type = msg_params->r.value[i++];

	for (n = 0; n < XA_REL_EQZ_FILTER_NUM; n++) {
		/* PRQA S 3440 5 */
		eqz_params->p_coef.type[n] = msg_params->r.value[i++];
		eqz_params->p_coef.fc[n] = msg_params->r.value[i++];
		eqz_params->p_coef.band_width[n] = msg_params->r.value[i++];
		eqz_params->p_coef.gain[n] = msg_params->r.value[i++];
		eqz_params->p_coef.gain_base[n] = msg_params->r.value[i++];
	}

	for (n = 0; n < XA_REL_EQZ_GRAPHIC_BAND_NUM; n++) {
		/* PRQA S 3440 1 */
		eqz_params->g_coef.gain_g[n] = msg_params->r.value[i++];
	}

exit:
	/* return msg to pool */
	xf_buffer_put(b);

	return err;
}

/** **************************************************************************
 *\brief  create Equalizer component
 *\internal
 *\covers: DD_DRV_CMN_01_019
 *
 *\param[in,out]  equalizer	 Pointer to the registered component
 *\param[in]	  cb		 Callback function
 *\param[in]	  private_data	 Private data
 *\retval	  0		 Success
 *\retval	  -EINVAL	 Invalid base instance or register fail
 *\retval	  -ENOMEM	 Cannot allocate Equalier instance
 *****************************************************************************/
int xf_adsp_equalizer_create(struct xf_adsp_equalizer **equalizer,
			     struct xf_callback_func *cb, void *private_data)
{
	struct xf_adsp_equalizer *eqz;
	int err;
	int comp_id;

	/* check the sane ADSP base data */
	if (!base)
		return -EINVAL;

	eqz = kmalloc(sizeof(*eqz), GFP_KERNEL);
	if (!eqz)
		return -ENOMEM;

	memset(eqz, 0, sizeof(struct xf_adsp_equalizer)); /* PRQA S 3200 */

	/* register equalizer component */
	err = xf_adsp_register("equalizer", &comp_id);
	if (err != 0)
		goto err2;			/* PRQA S 2001 */

	/* register equalizer to ADSP base control */
	eqz->handle_id = xf_adsp_base_register_handle(private_data,
						      cb, comp_id);

	if (eqz->handle_id <= 0) {
		err = -EINVAL;
		goto err1;			/* PRQA S 2001 */
	}

	/* get the default parameter from equalizer plugin */
	err = xf_adsp_equalizer_get_params(eqz);
	if (err != 0)
		goto err1;			/* PRQA S 2001 */

	/* save equalizer compoent data */
	*equalizer = eqz;

	return 0;

err1:
	xf_adsp_unregister(comp_id);		/* PRQA S 3200 */

err2:
	kfree(eqz);

	return err;
}

/** ***********************************************************
 *\brief  deinitialize ADSP Equalizer component
 *\internal
 *\covers: DD_DRV_CMN_01_020
 *
 *\param[in]	 equalizer	 Pointer to Equalizer component
 *\retval	 0		 Success
 *\retval	 -EINVAL	 Invalid base or Equalizer data
 **************************************************************/
int xf_adsp_equalizer_destroy(struct xf_adsp_equalizer *equalizer)
{
	struct xf_handle *handle;
	int handle_id;

	/* check the sane ADSP base and Equalizer data */
	if (!base || !equalizer)
		return -EINVAL;

	handle_id = equalizer->handle_id;

	handle = xf_adsp_base_get_handle(handle_id);
	if (!handle)
		goto exit;		/* PRQA S 2001 */

	/* unregister component */
	xf_adsp_unregister(handle->comp_id);	/* PRQA S 3200 */

	/* free handle data from base control */
	xf_adsp_base_free_handle(handle_id);	/* PRQA S 3200 */

exit:
	kfree(equalizer);

	return 0;
}

/***********************************************************************
 * APIs for TDM Renderer component
 * ********************************************************************/

/** ***********************************************************
 *\brief  set ADSP TDM Renderer parameters
 *\internal
 *\covers: DD_DRV_CMN_01_025
 *
 *\param[in]	 tdm_renderer	Pointer to TDM Renderer component
 *\retval	 0		Success
 *\retval	 -EINVAL	Failed
 **************************************************************/
int xf_adsp_tdm_renderer_set_params(struct xf_adsp_tdm_renderer *tdm_renderer)
{
	struct xf_message msg;
	struct xf_set_param_msg *msg_params;
	struct xf_adsp_tdm_renderer_params *params;
	struct xf_buffer *b;
	int i;
	struct xf_handle *handle;
	int err = 0;

	/* check the sane ADSP base data */
	if (!base || !tdm_renderer)
		return -EINVAL;

	params = &tdm_renderer->params;

	/* get TDM Renderer's handle data */
	handle = xf_adsp_base_get_handle(tdm_renderer->handle_id);
	if (!handle)
		return -EINVAL;

	b = xf_buffer_get(base->aux_pool);

	msg_params = xf_buffer_data(b);

	i = 0;
	msg_params->item[i].id = XA_TDM_RDR_CONFIG_PARAM_IN_SAMPLE_RATE;
	msg_params->item[i++].value = params->in_rate;

	msg_params->item[i].id = XA_TDM_RDR_CONFIG_PARAM_CHANNEL_MODE;
	msg_params->item[i++].value = params->ch_mode;

	msg_params->item[i].id = XA_TDM_RDR_CONFIG_PARAM_PCM_WIDTH;
	msg_params->item[i++].value = params->pcm_width;

	msg_params->item[i].id = XA_TDM_RDR_CONFIG_PARAM_FRAME_SIZE;
	msg_params->item[i++].value = params->frame_size;

	msg_params->item[i].id = XA_TDM_RDR_CONFIG_PARAM_OUTPUT1;
	msg_params->item[i++].value = params->dev1;

	msg_params->item[i].id = XA_TDM_RDR_CONFIG_PARAM_OUTPUT2;
	msg_params->item[i++].value = params->dev2;

	msg_params->item[i].id = XA_TDM_RDR_CONFIG_PARAM_DMACHANNEL1;
	msg_params->item[i++].value = params->dma1;

	msg_params->item[i].id = XA_TDM_RDR_CONFIG_PARAM_DMACHANNEL2;
	msg_params->item[i++].value = params->dma2;

	msg_params->item[i].id = XA_TDM_RDR_CONFIG_PARAM_OUT_SAMPLE_RATE;
	msg_params->item[i++].value = params->out_rate;

	msg_params->item[i].id = XA_TDM_RDR_CONFIG_PARAM_VOLUME_RATE;
	msg_params->item[i++].value = params->vol_rate;

	/* PRQA S 3200 2*/
	xf_create_msg(&msg,
		      __XF_MSG_ID(__XF_AP_CLIENT(0, tdm_renderer->handle_id),
				  __XF_PORT_SPEC2(handle->comp_id, 0)),
		      XF_SET_PARAM, XF_SET_PARAM_CMD_LEN(i), msg_params, NULL);

	err = xf_send_and_receive(&msg);

	/* return msg to pool */
	xf_buffer_put(b);

	return err;
}

/** ***********************************************************
 *\brief  get ADSP TDM Renderer parameters
 *\internal
 *\covers: DD_DRV_CMN_01_026
 *
 *\param[in]	 tdm_renderer	Pointer to TDM Renderer component
 *\retval	 0		Success
 *\retval	 -EINVAL	Failed
 **************************************************************/
int xf_adsp_tdm_renderer_get_params(struct xf_adsp_tdm_renderer *tdm_renderer)
{
	struct xf_adsp_tdm_renderer_params *params;
	struct xf_message msg;
	struct xf_buffer *b;
	union xf_get_param_msg *msg_params;
	int i;
	struct xf_handle *handle;
	int err = 0;

	/* check the sane ADSP base data */
	if (!base || !tdm_renderer)
		return -EINVAL;

	params = &tdm_renderer->params;

	/* get TDM Renderer's handle data */
	handle = xf_adsp_base_get_handle(tdm_renderer->handle_id);
	if (!handle)
		return -EINVAL;

	b = xf_buffer_get(base->aux_pool);
	msg_params = xf_buffer_data(b);

	i = 0;
	/* PRQA S 3440 13 1*/
	msg_params->c.id[i++] = XA_TDM_RDR_CONFIG_PARAM_IN_SAMPLE_RATE;
	msg_params->c.id[i++] = XA_TDM_RDR_CONFIG_PARAM_CHANNEL_MODE;
	msg_params->c.id[i++] = XA_TDM_RDR_CONFIG_PARAM_PCM_WIDTH;
	msg_params->c.id[i++] = XA_TDM_RDR_CONFIG_PARAM_FRAME_SIZE;
	msg_params->c.id[i++] = XA_TDM_RDR_CONFIG_PARAM_OUTPUT1;
	msg_params->c.id[i++] = XA_TDM_RDR_CONFIG_PARAM_OUTPUT2;
	msg_params->c.id[i++] = XA_TDM_RDR_CONFIG_PARAM_DMACHANNEL1;
	msg_params->c.id[i++] = XA_TDM_RDR_CONFIG_PARAM_DMACHANNEL2;
	msg_params->c.id[i++] = XA_TDM_RDR_CONFIG_PARAM_OUT_SAMPLE_RATE;
	msg_params->c.id[i++] = XA_TDM_RDR_CONFIG_PARAM_VOLUME_RATE;

	/* PRQA S 3200 2*/
	xf_create_msg(&msg,
		      __XF_MSG_ID(__XF_AP_CLIENT(0, tdm_renderer->handle_id),
				  __XF_PORT_SPEC2(handle->comp_id, 0)),
		      XF_GET_PARAM, XF_GET_PARAM_CMD_LEN(i), msg_params, NULL);

	err = xf_send_and_receive(&msg);
	if (err != 0)
		goto exit;		/* PRQA S 2001 */

	/* save the received parameters */
	i = 0;
	params->in_rate = msg_params->r.value[i++];	/* PRQA S 3440 13 */
	params->ch_mode = msg_params->r.value[i++];
	params->pcm_width = msg_params->r.value[i++];
	params->frame_size = msg_params->r.value[i++];
	params->dev1 = msg_params->r.value[i++];
	params->dev2 = msg_params->r.value[i++];
	params->dma1 = msg_params->r.value[i++];
	params->dma2 = msg_params->r.value[i++];
	params->out_rate = msg_params->r.value[i++];
	params->vol_rate = msg_params->r.value[i++];

exit:
	/* return msg to pool */
	xf_buffer_put(b);

	return err;
}

/** **************************************************************************
 *\brief  create TDM Renderer component
 *\internal
 *\covers: DD_DRV_CMN_01_023
 *
 *\param[in,out]  tdm_renderer	 Pointer to the registered component
 *\param[in]	  cb		 Callback function
 *\param[in]	  private_data	 Private data
 *\retval	  0		 Success
 *\retval	  -EINVAL	 Invalid base instance or register fail
 *\retval	  -ENOMEM	 Cannot allocate Renderer instance
 *****************************************************************************/
int xf_adsp_tdm_renderer_create(struct xf_adsp_tdm_renderer **tdm_renderer,
				struct xf_callback_func *cb,
				void *private_data)
{
	struct xf_adsp_tdm_renderer *tdm_rdr;
	int err;
	int comp_id;

	/* check the sane ADSP base data */
	if (!base)
		return -EINVAL;

	tdm_rdr = kmalloc(sizeof(*tdm_rdr), GFP_KERNEL);
	if (!tdm_rdr)
		return -ENOMEM;

	/* PRQA S 3200 */
	memset(tdm_rdr, 0, sizeof(struct xf_adsp_tdm_renderer));

	/* register TDM Renderer component */
	err = xf_adsp_register("tdm-renderer", &comp_id);
	if (err != 0)
		goto err2;		/* PRQA S 2001 */

	/* register TDM Renderer to ADSP base control */
	tdm_rdr->handle_id = xf_adsp_base_register_handle(private_data,
							  cb, comp_id);

	if (tdm_rdr->handle_id <= 0) {
		err = -EINVAL;
		goto err1;		/* PRQA S 2001 */
	}

	/* get the default parameter from plugin */
	err = xf_adsp_tdm_renderer_get_params(tdm_rdr);
	if (err != 0)
		goto err1;		/* PRQA S 2001 */

	/* save compoent data */
	*tdm_renderer = tdm_rdr;

	return 0;

err1:
	xf_adsp_unregister(comp_id);		/* PRQA S 3200 */

err2:
	kfree(tdm_rdr);

	return err;
}

/** ***********************************************************
 *\brief  deinitialize ADSP TDM Renderer component
 *\internal
 *\covers: DD_DRV_CMN_01_024
 *
 *\param[in]	 tdm_renderer	Pointer to TDM Renderer component
 *\retval	 0		Success
 *\retval	 -EINVAL	Invalid base or Renderer data
 **************************************************************/
int xf_adsp_tdm_renderer_destroy(struct xf_adsp_tdm_renderer *tdm_renderer)
{
	struct xf_handle *handle;
	int handle_id;

	/* check the sane ADSP base data */
	if (!base || !tdm_renderer)
		return -EINVAL;

	handle_id = tdm_renderer->handle_id;

	handle = xf_adsp_base_get_handle(handle_id);
	if (!handle)
		goto exit;		/* PRQA S 2001 */

	/* unregister component */
	xf_adsp_unregister(handle->comp_id);		/* PRQA S 3200 */

	/* free handle data from base control */
	xf_adsp_base_free_handle(handle_id);		/* PRQA S 3200 */

exit:
	kfree(tdm_renderer);

	return 0;
}

/***********************************************************************
 * APIs for TDM Capture component
 * ********************************************************************/

/** ***********************************************************
 *\brief  set ADSP TDM Capture parameters
 *\internal
 *\covers: DD_DRV_CMN_01_029
 *
 *\param[in]	 tdm_capture	Pointer to TDM Capture component
 *\retval	 0		Success
 *\retval	 -EINVAL	Failed
 **************************************************************/
int xf_adsp_tdm_capture_set_params(struct xf_adsp_tdm_capture *tdm_capture)
{
	struct xf_message msg;
	struct xf_set_param_msg *msg_params;
	struct xf_adsp_tdm_capture_params *params;
	struct xf_buffer *b;
	int i;
	struct xf_handle *handle;
	int err = 0;

	/* check the sane ADSP base data */
	if (!base || !tdm_capture)
		return -EINVAL;

	params = &tdm_capture->params;

	/* get TDM Capture's handle data */
	handle = xf_adsp_base_get_handle(tdm_capture->handle_id);
	if (!handle)
		return -EINVAL;

	b = xf_buffer_get(base->aux_pool);

	msg_params = xf_buffer_data(b);

	i = 0;
	msg_params->item[i].id = XA_TDM_CAP_CONFIG_PARAM_IN_SAMPLE_RATE;
	msg_params->item[i++].value = params->in_rate;

	msg_params->item[i].id = XA_TDM_CAP_CONFIG_PARAM_CHANNEL_MODE;
	msg_params->item[i++].value = params->ch_mode;

	msg_params->item[i].id = XA_TDM_CAP_CONFIG_PARAM_PCM_WIDTH;
	msg_params->item[i++].value = params->pcm_width;

	msg_params->item[i].id = XA_TDM_CAP_CONFIG_PARAM_FRAME_SIZE;
	msg_params->item[i++].value = params->frame_size;

	msg_params->item[i].id = XA_TDM_CAP_CONFIG_PARAM_INPUT1;
	msg_params->item[i++].value = params->dev1;

	msg_params->item[i].id = XA_TDM_CAP_CONFIG_PARAM_INPUT2;
	msg_params->item[i++].value = params->dev2;

	msg_params->item[i].id = XA_TDM_CAP_CONFIG_PARAM_DMACHANNEL1;
	msg_params->item[i++].value = params->dma1;

	msg_params->item[i].id = XA_TDM_CAP_CONFIG_PARAM_DMACHANNEL2;
	msg_params->item[i++].value = params->dma2;

	msg_params->item[i].id = XA_TDM_CAP_CONFIG_PARAM_OUT_SAMPLE_RATE;
	msg_params->item[i++].value = params->out_rate;

	msg_params->item[i].id = XA_TDM_CAP_CONFIG_PARAM_VOLUME_RATE;
	msg_params->item[i++].value = params->vol_rate;

	/* PRQA S 3200 2*/
	xf_create_msg(&msg,
		      __XF_MSG_ID(__XF_AP_CLIENT(0, tdm_capture->handle_id),
				  __XF_PORT_SPEC2(handle->comp_id, 0)),
		      XF_SET_PARAM, XF_SET_PARAM_CMD_LEN(i), msg_params, NULL);

	err = xf_send_and_receive(&msg);

	/* return msg to pool */
	xf_buffer_put(b);

	return err;
}

/** ***********************************************************
 *\brief  get ADSP TDM Capture parameters
 *\internal
 *\covers: DD_DRV_CMN_01_030
 *
 *\param[in]	 tdm_capture	Pointer to TDM Capture component
 *\retval	 0		Success
 *\retval	 -EINVAL	Failed
 **************************************************************/
int xf_adsp_tdm_capture_get_params(struct xf_adsp_tdm_capture *tdm_capture)
{
	struct xf_adsp_tdm_capture_params *params;
	struct xf_message msg;
	struct xf_buffer *b;
	union xf_get_param_msg *msg_params;
	int i;
	struct xf_handle *handle;
	int err = 0;

	/* check the sane ADSP base data */
	if (!base || !tdm_capture)
		return -EINVAL;

	params = &tdm_capture->params;

	/* get TDM Capture's handle data */
	handle = xf_adsp_base_get_handle(tdm_capture->handle_id);
	if (!handle)
		return -EINVAL;

	b = xf_buffer_get(base->aux_pool);
	msg_params = xf_buffer_data(b);

	i = 0;
	/* PRQA S 3440 13 1*/
	msg_params->c.id[i++] = XA_TDM_CAP_CONFIG_PARAM_IN_SAMPLE_RATE;
	msg_params->c.id[i++] = XA_TDM_CAP_CONFIG_PARAM_CHANNEL_MODE;
	msg_params->c.id[i++] = XA_TDM_CAP_CONFIG_PARAM_PCM_WIDTH;
	msg_params->c.id[i++] = XA_TDM_CAP_CONFIG_PARAM_FRAME_SIZE;
	msg_params->c.id[i++] = XA_TDM_CAP_CONFIG_PARAM_INPUT1;
	msg_params->c.id[i++] = XA_TDM_CAP_CONFIG_PARAM_INPUT2;
	msg_params->c.id[i++] = XA_TDM_CAP_CONFIG_PARAM_DMACHANNEL1;
	msg_params->c.id[i++] = XA_TDM_CAP_CONFIG_PARAM_DMACHANNEL2;
	msg_params->c.id[i++] = XA_TDM_CAP_CONFIG_PARAM_OUT_SAMPLE_RATE;
	msg_params->c.id[i++] = XA_TDM_CAP_CONFIG_PARAM_VOLUME_RATE;

	/* PRQA S 3200 2*/
	xf_create_msg(&msg,
		      __XF_MSG_ID(__XF_AP_CLIENT(0, tdm_capture->handle_id),
				  __XF_PORT_SPEC2(handle->comp_id, 0)),
		      XF_GET_PARAM, XF_GET_PARAM_CMD_LEN(i), msg_params, NULL);

	err = xf_send_and_receive(&msg);
	if (err != 0)
		goto exit;		/* PRQA S 2001 */

	/* save the received parameters */
	i = 0;
	params->in_rate = msg_params->r.value[i++];	/* PRQA S 3440 13 */
	params->ch_mode = msg_params->r.value[i++];
	params->pcm_width = msg_params->r.value[i++];
	params->frame_size = msg_params->r.value[i++];
	params->dev1 = msg_params->r.value[i++];
	params->dev2 = msg_params->r.value[i++];
	params->dma1 = msg_params->r.value[i++];
	params->dma2 = msg_params->r.value[i++];
	params->out_rate = msg_params->r.value[i++];
	params->vol_rate = msg_params->r.value[i++];

exit:
	/* return msg to pool */
	xf_buffer_put(b);

	return err;
}

/** **************************************************************************
 *\brief  create TDM Capture component
 *\internal
 *\covers: DD_DRV_CMN_01_027
 *
 *\param[in,out]  tdm_capture	 Pointer to the registered component
 *\param[in]	  cb		 Callback function
 *\param[in]	  private_data	 Private data
 *\retval	  0		 Success
 *\retval	  -EINVAL	 Invalid base instance or register fail
 *\retval	  -ENOMEM	 Cannot allocate Renderer instance
 *****************************************************************************/
int xf_adsp_tdm_capture_create(struct xf_adsp_tdm_capture **tdm_capture,
			       struct xf_callback_func *cb, void *private_data)
{
	struct xf_adsp_tdm_capture *tdm_cap;
	int err;
	int comp_id;

	/* check the sane ADSP base data */
	if (!base)
		return -EINVAL;

	tdm_cap = kmalloc(sizeof(*tdm_cap), GFP_KERNEL);
	if (!tdm_cap)
		return -ENOMEM;

	/* PRQA S 3200 */
	memset(tdm_cap, 0, sizeof(struct xf_adsp_tdm_capture));

	/* register TDM Capture component */
	err = xf_adsp_register("tdm-capture", &comp_id);
	if (err != 0)
		goto err2;		/* PRQA S 2001 */

	/* register TDM Capture to ADSP base control */
	tdm_cap->handle_id = xf_adsp_base_register_handle(private_data,
							  cb, comp_id);

	if (tdm_cap->handle_id <= 0) {
		err = -EINVAL;
		goto err1;		/* PRQA S 2001 */
	}

	/* get the default parameter from plugin */
	err = xf_adsp_tdm_capture_get_params(tdm_cap);
	if (err != 0)
		goto err1;		/* PRQA S 2001 */

	/* save compoent data */
	*tdm_capture = tdm_cap;

	return 0;

err1:
	xf_adsp_unregister(comp_id);		/* PRQA S 3200 */

err2:
	kfree(tdm_cap);

	return err;
}

/** ***********************************************************
 *\brief  deinitialize ADSP TDM Capture component
 *\internal
 *\covers: DD_DRV_CMN_01_028
 *
 *\param[in]	 tdm_capture	 Pointer to TDM Capture component
 *\retval	 0		 Success
 *\retval	 -EINVAL	 Invalid base or Renderer data
 **************************************************************/
int xf_adsp_tdm_capture_destroy(struct xf_adsp_tdm_capture *tdm_capture)
{
	struct xf_handle *handle;
	int handle_id;

	/* check the sane ADSP base data */
	if (!base || !tdm_capture)
		return -EINVAL;

	handle_id = tdm_capture->handle_id;

	handle = xf_adsp_base_get_handle(handle_id);
	if (!handle)
		goto exit;		/* PRQA S 2001 */

	/* unregister component */
	xf_adsp_unregister(handle->comp_id);		/* PRQA S 3200 */

	/* free handle data from base control */
	xf_adsp_base_free_handle(handle_id);		/* PRQA S 3200 */

exit:
	kfree(tdm_capture);

	return 0;
}
