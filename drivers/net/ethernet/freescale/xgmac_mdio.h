#include <linux/kernel.h>
#include <linux/mdio.h>

struct mdio_fsl_priv {
	void __iomem *map;
	struct	fsl_mdio_regs __iomem *mdio_base;
	bool	is_little_endian;
};

struct xgmac_mdio_cfg {
	unsigned int regs_offset;
	const char *bus_name;
};

int xgmac_mdio_probe(struct device *dev, struct resource *res,
		     const struct xgmac_mdio_cfg *cfg);

int xgmac_mdio_remove(struct mii_bus *bus);
