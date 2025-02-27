/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2009-2010 Advanced Micro Devices, Inc.
 * Author: Joerg Roedel <jroedel@suse.de>
 */

#ifndef AMD_IOMMU_H
#define AMD_IOMMU_H

#include <linux/iommu.h>

#include "amd_iommu_types.h"

void iommu_reset_vmmio(struct amd_iommu *iommu, u16 guestId);
irqreturn_t amd_iommu_int_thread(int irq, void *data);
irqreturn_t amd_iommu_int_thread_evtlog(int irq, void *data);
irqreturn_t amd_iommu_int_thread_pprlog(int irq, void *data);
irqreturn_t amd_iommu_int_thread_galog(int irq, void *data);
irqreturn_t amd_iommu_int_handler(int irq, void *data);
void amd_iommu_restart_event_logging(struct amd_iommu *iommu);
void amd_iommu_restart_ga_log(struct amd_iommu *iommu);
void amd_iommu_restart_ppr_log(struct amd_iommu *iommu);
void amd_iommu_set_rlookup_table(struct amd_iommu *iommu, u16 devid);
void iommu_feature_enable(struct amd_iommu *iommu, u8 bit);
bool iommu_feature_enable_and_check(struct amd_iommu *iommu, u8 bit);
int amd_iommu_pdom_id_alloc(void);
void amd_iommu_pdom_id_free(int id);
u8 __iomem * __init iommu_map_mmio_space(u64 address, u64 end);
int iommu_flush_dte(struct amd_iommu *iommu, u16 devid);
struct iommu_domain *amd_iommu_domain_alloc(unsigned int type);
int amd_iommu_v1_map_pages(struct io_pgtable_ops *ops, unsigned long iova,
			   phys_addr_t paddr, size_t pgsize, size_t pgcount,
			   int prot, gfp_t gfp, size_t *mapped);
unsigned long amd_iommu_v1_unmap_pages(struct io_pgtable_ops *ops,
				       unsigned long iova,
				       size_t pgsize, size_t pgcount,
				       struct iommu_iotlb_gather *gather);

#ifdef CONFIG_AMD_IOMMU_DEBUGFS
void amd_iommu_debugfs_setup(void);
#else
static inline void amd_iommu_debugfs_setup(void) {}
#endif

extern bool amd_iommu_viommu;
extern const struct amd_iommu_svm_ops *svm_ops;

/* Needed for interrupt remapping */
int amd_iommu_prepare(void);
int amd_iommu_enable(void);
void amd_iommu_disable(void);
int amd_iommu_reenable(int mode);
int amd_iommu_enable_faulting(void);
extern int amd_iommu_guest_ir;
extern enum io_pgtable_fmt amd_iommu_pgtable;
extern int amd_iommu_gpt_level;

struct protection_domain *protection_domain_alloc(unsigned int type);
void amd_iommu_domain_free(struct iommu_domain *dom);

bool amd_iommu_v2_supported(void);
struct amd_iommu *get_amd_iommu(unsigned int idx);
u8 amd_iommu_pc_get_max_banks(unsigned int idx);
bool amd_iommu_pc_supported(void);
u8 amd_iommu_pc_get_max_counters(unsigned int idx);
int amd_iommu_pc_get_reg(struct amd_iommu *iommu, u8 bank, u8 cntr,
			 u8 fxn, u64 *value);
int amd_iommu_pc_set_reg(struct amd_iommu *iommu, u8 bank, u8 cntr,
			 u8 fxn, u64 *value);

void amd_iommu_init_identity_domain(void);

/* Device capabilities */
int amd_iommu_pdev_enable_cap_pri(struct pci_dev *pdev);
void amd_iommu_pdev_disable_cap_pri(struct pci_dev *pdev);

/* GCR3 setup */
int amd_iommu_set_gcr3(struct iommu_dev_data *dev_data,
		       ioasid_t pasid, unsigned long gcr3);
int amd_iommu_clear_gcr3(struct iommu_dev_data *dev_data, ioasid_t pasid);

/*
 * This function flushes all internal caches of
 * the IOMMU used by this driver.
 */
void amd_iommu_flush_all_caches(struct amd_iommu *iommu);
void amd_iommu_update_and_flush_device_table(struct protection_domain *domain);
void amd_iommu_domain_update(struct protection_domain *domain);
void amd_iommu_domain_flush_complete(struct protection_domain *domain);
void amd_iommu_domain_flush_all(struct protection_domain *domain);
void amd_iommu_domain_flush_pages(struct protection_domain *domain,
				  u64 address, size_t size);
void amd_iommu_dev_flush_pasid_pages(struct iommu_dev_data *dev_data,
				     ioasid_t pasid, u64 address, size_t size);
void amd_iommu_dev_flush_pasid_all(struct iommu_dev_data *dev_data,
				   ioasid_t pasid);

void amd_iommu_iotlb_sync(struct iommu_domain *domain,
			  struct iommu_iotlb_gather *gather);
int amd_iommu_flush_private_vm_region(struct amd_iommu *iommu, struct protection_domain *pdom,
				      u64 address, size_t size);

#ifdef CONFIG_IRQ_REMAP
int amd_iommu_create_irq_domain(struct amd_iommu *iommu);
#else
static inline int amd_iommu_create_irq_domain(struct amd_iommu *iommu)
{
	return 0;
}
#endif

#define PPR_SUCCESS			0x0
#define PPR_INVALID			0x1
#define PPR_FAILURE			0xf

int amd_iommu_complete_ppr(struct pci_dev *pdev, u32 pasid,
			   int status, int tag);

static inline bool is_rd890_iommu(struct pci_dev *pdev)
{
	return (pdev->vendor == PCI_VENDOR_ID_ATI) &&
	       (pdev->device == PCI_DEVICE_ID_RD890_IOMMU);
}

static inline bool check_feature(u64 mask)
{
	return (amd_iommu_efr & mask);
}

static inline bool check_feature2(u64 mask)
{
	return (amd_iommu_efr2 & mask);
}

static inline bool amd_iommu_v2_pgtbl_supported(void)
{
	return (check_feature(FEATURE_GIOSUP) && check_feature(FEATURE_GT));
}

static inline bool amd_iommu_gt_ppr_supported(void)
{
	return (amd_iommu_v2_pgtbl_supported() &&
		check_feature(FEATURE_PPR) &&
		check_feature(FEATURE_EPHSUP));
}

static inline u64 iommu_virt_to_phys(void *vaddr)
{
	return (u64)__sme_set(virt_to_phys(vaddr));
}

static inline void *iommu_phys_to_virt(unsigned long paddr)
{
	return phys_to_virt(__sme_clr(paddr));
}

static inline
void amd_iommu_domain_set_pt_root(struct protection_domain *domain, u64 root)
{
	domain->iop.root = (u64 *)(root & PAGE_MASK);
	domain->iop.mode = root & 7; /* lowest 3 bits encode pgtable mode */
}

static inline
void amd_iommu_domain_clr_pt_root(struct protection_domain *domain)
{
	amd_iommu_domain_set_pt_root(domain, 0);
}

static inline int get_pci_sbdf_id(struct pci_dev *pdev)
{
	int seg = pci_domain_nr(pdev->bus);
	u16 devid = pci_dev_id(pdev);

	return PCI_SEG_DEVID_TO_SBDF(seg, devid);
}

static inline void *alloc_pgtable_page(int nid, gfp_t gfp)
{
	struct page *page;

	page = alloc_pages_node(nid, gfp | __GFP_ZERO, 0);
	return page ? page_address(page) : NULL;
}

/*
 * This must be called after device probe completes. During probe
 * use rlookup_amd_iommu() get the iommu.
 */
static inline struct amd_iommu *get_amd_iommu_from_dev(struct device *dev)
{
	return iommu_get_iommu_dev(dev, struct amd_iommu, iommu);
}

/* This must be called after device probe completes. */
static inline struct amd_iommu *get_amd_iommu_from_dev_data(struct iommu_dev_data *dev_data)
{
	return iommu_get_iommu_dev(dev_data->dev, struct amd_iommu, iommu);
}

//SURAVEE: TODO: This is a hack. Remove this
static inline struct amd_iommu *get_amd_iommu_from_devid(u16 devid)
{
	struct amd_iommu *iommu;

	for_each_iommu(iommu)
		if (iommu->devid == devid)
			return iommu;
	return NULL;
}

static inline struct protection_domain *to_pdomain(struct iommu_domain *dom)
{
	return container_of(dom, struct protection_domain, domain);
}

static inline size_t get_irq_table_size(unsigned int max_irqs)
{
	if (!AMD_IOMMU_GUEST_IR_GA(amd_iommu_guest_ir))
		return max_irqs * sizeof(u32);

	return max_irqs * (sizeof(u64) * 2);
}

bool translation_pre_enabled(struct amd_iommu *iommu);
bool amd_iommu_is_attach_deferred(struct device *dev);
int __init add_special_device(u8 type, u8 id, u32 *devid, bool cmd_line);

#ifdef CONFIG_DMI
void amd_iommu_apply_ivrs_quirks(void);
#else
static inline void amd_iommu_apply_ivrs_quirks(void) { }
#endif

void amd_iommu_domain_set_pgtable(struct protection_domain *domain,
				  u64 *root, int mode);
struct dev_table_entry *get_dev_table(struct amd_iommu *iommu);

extern bool amd_iommu_snp_en;

struct dev_table_entry *amd_iommu_get_ivhd_dte_flags(u16 segid, u16 devid);
struct iommu_dev_data *search_dev_data(struct amd_iommu *iommu, u16 devid);
int amd_iommu_completion_wait(struct amd_iommu *iommu);

void amd_iommu_set_dte_v1(struct iommu_dev_data *dev_data,
			  struct protection_domain *domain,
			  u16 domid, struct dev_table_entry *new);

void amd_iommu_update_dte(struct amd_iommu *iommu,
			  struct iommu_dev_data *dev_data,
			  struct dev_table_entry *new);
void amd_iommu_set_translate_dte(struct amd_iommu *iommu, u16 gid,
				 struct protection_domain *pdom,
				 u32 devid);
void amd_iommu_clear_translate_dte(struct amd_iommu *iommu, u16 gid, u32 devid);
void amd_iommu_update_vfctrl_mmio_translate_devid(struct amd_iommu *iommu,
						  u16 gid, u32 trans_devid);

static inline void
amd_iommu_make_clear_dte(struct iommu_dev_data *dev_data, struct dev_table_entry *new)
{
	struct dev_table_entry *initial_dte;
	struct amd_iommu *iommu = get_amd_iommu_from_dev(dev_data->dev);

	/* All existing DTE must have V bit set */
	new->data128[0] = DTE_FLAG_V;
	new->data128[1] = 0;

	/*
	 * Restore cached persistent DTE bits, which can be set by information
	 * in IVRS table. See set_dev_entry_from_acpi().
	 */
	initial_dte = amd_iommu_get_ivhd_dte_flags(iommu->pci_seg->id, dev_data->devid);
	if (initial_dte) {
		new->data128[0] |= initial_dte->data128[0];
		new->data128[1] |= initial_dte->data128[1];
	}
}

/* NESTED */
struct iommu_domain *
amd_iommu_alloc_domain_nested(struct iommufd_viommu *viommu, u32 flags,
			      const struct iommu_user_data *user_data);
/* Guest ID for vIOMMU */
int amd_iommu_gid_alloc(void);
void amd_iommu_gid_free(int gid);

#endif /* AMD_IOMMU_H */
