/*
 * Copyright (C) 2018 GlobalLogic
*/

#include <linux/uaccess.h>
#include <linux/dma-buf.h>
#include <asm/memory.h>
#include <linux/err.h>
#include "ion.h"

long rcar_ion_get_phys_addr(unsigned long arg)
{
	struct dma_buf *dmabuf;
	struct ion_buffer *buffer = NULL;
	struct ion_custom_data data;
	phys_addr_t paddr;

	if (copy_from_user(&data, (void __user *)arg,
			sizeof(struct ion_custom_data)))
		return -EFAULT;

	dmabuf = dma_buf_get(data.cmd);
	if (IS_ERR(dmabuf))
		return PTR_ERR(dmabuf);

	buffer = (struct ion_buffer *)dmabuf->priv;
	if (IS_ERR_OR_NULL(buffer) || !buffer->size) {
		pr_err("%s: Not initialized or empty buffer.\n", __FUNCTION__);
		return -EFAULT;
	}

	paddr = page_to_phys((struct page *)buffer->priv_virt);
	dma_buf_put(dmabuf);

	data.arg = (unsigned long)paddr;

	if (copy_to_user((void __user *)arg, &data,
				sizeof(struct ion_custom_data)))
		return -EFAULT;

	return 0;
}
