/*
 * Copyright (C) 2018 GlobalLogic
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/uaccess.h>
#include <linux/dma-buf.h>
#include <linux/genalloc.h>
#include <asm/memory.h>
#include <linux/err.h>
#include <linux/slab.h>

#include "ion.h"
#include "ion_of.h"

static struct ion_of_heap rcar_ion_heaps[] = {
	PLATFORM_HEAP("renesas,ion-rcar-heap", ION_HEAP_TYPE_DMA,
		ION_HEAP_TYPE_CARVEOUT, "ion_heap"),
};

struct ion_rcar_heap {
	struct ion_heap heap;
	struct gen_pool *pool;
	phys_addr_t base;
};

long rcar_ion_get_phys_addr(unsigned long arg)
{
	struct dma_buf *dmabuf;
	struct ion_buffer *buffer = NULL;
	struct ion_custom_data data;
	struct page *page;
	phys_addr_t paddr;

	if (copy_from_user(&data, (void __user *)arg,
			sizeof(struct ion_custom_data)))
		return -EFAULT;

	dmabuf = dma_buf_get(data.cmd);
	if (IS_ERR(dmabuf))
		return PTR_ERR(dmabuf);

	buffer = (struct ion_buffer *)dmabuf->priv;
	if (IS_ERR_OR_NULL(buffer) || !buffer->size) {
		pr_err("%s: Not initialized or empty buffer.\n", __func__);
		return -EFAULT;
	}

	page = sg_page(buffer->sg_table->sgl);
	paddr = page_to_phys(page);

	dma_buf_put(dmabuf);

	data.arg = (unsigned long)paddr;
	if (copy_to_user((void __user *)arg, &data,
				sizeof(struct ion_custom_data)))
		return -EFAULT;

	return 0;
}

static int rcar_ion_heap_allocate(struct ion_heap *heap,
		struct ion_buffer *buffer, unsigned long size, unsigned long flags)
{
	struct ion_rcar_heap *rheap = container_of(heap, struct ion_rcar_heap, heap);
	struct sg_table *table;
	phys_addr_t paddr;
	int ret;
	struct page *page;

	table = kmalloc(sizeof(*table), GFP_KERNEL);
	if (!table)
		return -ENOMEM;
	ret = sg_alloc_table(table, 1, GFP_KERNEL);
	if (ret)
		goto err_free;

	paddr = gen_pool_alloc(rheap->pool, size);

	if (!paddr) {
		ret = -ENOMEM;
		goto err_free_table;
	}

	sg_set_page(table->sgl, pfn_to_page(PFN_DOWN(paddr)), size, 0);
	buffer->sg_table = table;

	page = sg_page(buffer->sg_table->sgl);
	if (page) {
		mod_node_page_state(page_pgdat(page), NR_ION_HEAP, (size/PAGE_SIZE));
		mod_node_page_state(page_pgdat(page), NR_ION_HEAP_POOL, -(size/PAGE_SIZE));
	}

	return 0;

err_free_table:
	sg_free_table(table);
err_free:
	kfree(table);
	return ret;
}

static void rcar_ion_heap_free(struct ion_buffer *buffer)
{
	struct ion_heap *heap = buffer->heap;
	struct sg_table *table = buffer->sg_table;
	struct page *page = sg_page(table->sgl);
	struct ion_rcar_heap *rheap = container_of(heap, struct ion_rcar_heap, heap);
	phys_addr_t paddr = PFN_PHYS(page_to_pfn(page));

	ion_heap_buffer_zero(buffer);

	if (paddr == -1)
		return;

	gen_pool_free(rheap->pool, paddr, buffer->size);
	sg_free_table(table);
	kfree(table);
	mod_node_page_state(page_pgdat(page), NR_ION_HEAP, -(buffer->size/PAGE_SIZE));
	mod_node_page_state(page_pgdat(page), NR_ION_HEAP_POOL, buffer->size/PAGE_SIZE);
}

static struct ion_heap_ops rcar_ion_heap_ops = {
	.allocate           = rcar_ion_heap_allocate,
	.free               = rcar_ion_heap_free,
	.map_user           = ion_heap_map_user,
	.map_kernel         = ion_heap_map_kernel,
	.unmap_kernel       = ion_heap_unmap_kernel,
};

/* ------------------------------------------------------------------ */
int rcar_ion_probe(struct platform_device *pdev)
{
	struct ion_platform_data *pdata;
	struct ion_platform_heap *pheap;
	struct ion_rcar_heap *rheap;
	struct page *page;
	size_t size;
	int ret;

	rheap = kzalloc(sizeof(*rheap), GFP_KERNEL);
	if (!rheap)
		return -ENOMEM;

	pdata = ion_parse_dt(pdev, rcar_ion_heaps);
	if (IS_ERR(pdata)) {
		ret = PTR_ERR(pdata);
		dev_err(&pdev->dev, "ion_parse_dt failed, err=%d\n", ret);
		goto err_parse_dt;
	}

	if(!(pdata->nr > 0)) {
		dev_err(&pdev->dev, "Least one ION heap must be\n");
		ret = -EINVAL;
		goto err_probe;
	}

	pheap = &pdata->heaps[0];
	page = pfn_to_page(PFN_DOWN(pheap->base));
	size = pheap->size;

	ret = ion_heap_pages_zero(page, size, pgprot_writecombine(PAGE_KERNEL));
	if (ret) {
		dev_err(&pdev->dev, "ion_heap_pages_zero failed, err=%d\n", ret);
		goto err_probe;
	}

	rheap->pool = gen_pool_create(PAGE_SHIFT, -1);
	if (!rheap->pool) {
		dev_err(&pdev->dev, "gen_pool_create failed, out of memory?\n");
		ret = -ENOMEM;
		goto err_probe;
	}

	gen_pool_add(rheap->pool, pheap->base, pheap->size, -1);

	mod_node_page_state(page_pgdat(page), NR_ION_HEAP_POOL,
			    pheap->size/PAGE_SIZE);

	rheap->base = pheap->base;
	rheap->heap.ops = &rcar_ion_heap_ops;
	rheap->heap.type = ION_HEAP_TYPE_DMA;
	rheap->heap.flags = ION_HEAP_FLAG_DEFER_FREE;
	rheap->heap.name = "rcar-ion";

	ion_device_add_heap(&rheap->heap);

	dev_info(&pdev->dev, "heap '%s' base 0x%pa size %luKB id %d type %d\n",
		rheap->heap.name, &pheap->base, pheap->size/1024, pheap->id, pheap->type);

	return 0;

err_probe:
	ion_destroy_platform_data(pdata);
err_parse_dt:
	kfree(rheap);
	return ret;
}

int rcar_ion_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id rcar_ion_of_table[] = {
	{ .compatible = "renesas,ion-rcar", },
	{ },
};
MODULE_DEVICE_TABLE(of, rcar_ion_of_table);

static struct platform_driver ion_driver = {
	.probe = rcar_ion_probe,
	.remove = rcar_ion_remove,
	.driver = {
		.name = "rcar-ion",
		.of_match_table = rcar_ion_of_table,
		.owner = THIS_MODULE
	 },
};
module_platform_driver(ion_driver);

MODULE_DESCRIPTION("R-CAR platform ION allocator");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:ion-rcar");
