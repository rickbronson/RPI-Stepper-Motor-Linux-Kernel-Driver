struct bcm2835_cb_entry {
	struct bcm2835_dma_cb *cb;
	dma_addr_t paddr;
};

/* taken from linux/drivers/dma/virt-dma.h */
struct virt_dma_desc {
	struct dma_async_tx_descriptor tx;
	struct dmaengine_result tx_result;
	/* protected by vc.lock */
	struct list_head node;
};

struct virt_dma_chan {
	struct dma_chan	chan;
	struct tasklet_struct task;
	void (*desc_free)(struct virt_dma_desc *);

	spinlock_t lock;

	/* protected by vc.lock */
	struct list_head desc_allocated;
	struct list_head desc_submitted;
	struct list_head desc_issued;
	struct list_head desc_completed;

	struct virt_dma_desc *cyclic;
	struct virt_dma_desc *vd_terminated;
};

struct bcm2835_desc {
	struct bcm2835_chan *c;
	struct virt_dma_desc vd;
	enum dma_transfer_direction dir;

	unsigned int frames;
	size_t size;

	bool cyclic;

	struct bcm2835_cb_entry cb_list[];
};

struct bcm2835_chan {
	struct virt_dma_chan vc;

	struct dma_slave_config	cfg;
	unsigned int dreq;

	int ch;
	struct bcm2835_desc *desc;
	struct dma_pool *cb_pool;

	void __iomem *chan_base;
	int irq_number;
	unsigned int irq_flags;

	bool is_lite_channel;
	bool is_40bit_channel;
};

static inline struct bcm2835_chan *to_bcm2835_dma_chan(struct dma_chan *c)
{
	return container_of(c, struct bcm2835_chan, vc.chan);
}
