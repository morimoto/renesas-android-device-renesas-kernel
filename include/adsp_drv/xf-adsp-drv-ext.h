/*****************************************************************************
 * \file       xf-adsp-driver-ext.h
 * \brief      Header file for ADSP driver extension part
 * \addtogroup ADSP Driver
 ******************************************************************************
 * \date       Oct. 21, 2017
 * \author     Renesas Electronics Corporation
 ******************************************************************************
 * \par        Copyright
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

#ifndef __XF_ADSP_DRV_EXT_H
#define __XF_ADSP_DRV_EXT_H

/***********************************************************
 * Extension client APIs
 * ********************************************************/

struct xf_adsp_base_cmd {
	/* register new client for ADSP base control */
	int (*client_register)(void **private_data);
	/* unregister client */
	int (*client_unregister)(void *private_data);
	/* get data from proxy */
	int (*recv)(void *private_data, void *buf);
	/* send data to proxy */
	int (*send)(void *private_data, void *buf);
	/* wait the valid message in the response queue */
	int (*poll)(void *private_data, int *condition);
};

/* create ADSP base control data */
int xf_adsp_base_create(struct xf_adsp_base_cmd *cmd);

/* destroy ADSP base control data */
int xf_adsp_base_destroy(void);

#endif
