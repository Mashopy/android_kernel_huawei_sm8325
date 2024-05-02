

#include <linux/interconnect.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/seq_file.h>
#include <linux/proc_fs.h>
#include <linux/fs.h>
 
static struct platform_device *icc_pdev;
static struct icc_path *i_path;
 
#define MLEVEL_BW_UTILIZATION 1
extern unsigned int enable_cx_decrease;
static unsigned long enable_bw = 0;
 
static ssize_t bw_proc_write(struct file *file, char const __user *buf,
							size_t count, loff_t *ppos)
{
	int ret;
 
	ret = kstrtoul_from_user(buf, count, 10, &enable_bw);
	pr_err("enable_bw_optimize: enable_bw = %lu, count = %d\n", enable_bw, count);

	if (ret)
		return ret;

	if (enable_bw == MLEVEL_BW_UTILIZATION)
		enable_cx_decrease = MLEVEL_BW_UTILIZATION;
	else
		enable_cx_decrease = 0;

	*ppos += count;

	return count;
}
 
static int bw_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", enable_bw);
	return 0;
}
 
static int bw_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, bw_proc_show, NULL);
}
 
static const struct file_operations bw_opt_fops = {
	.owner = THIS_MODULE,
	.open = bw_proc_open,
	.write = bw_proc_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};
 
static void __exit icc_opt_exit(void)
{
	if (!IS_ERR(i_path))
		icc_put(i_path);
 
	platform_device_del(icc_pdev);
	platform_device_put(icc_pdev);
}
 
static int __init icc_opt_init(void)
{
	icc_pdev = platform_device_alloc("icc-opt", PLATFORM_DEVID_AUTO);
	platform_device_add(icc_pdev);
 
	if (!proc_create("enable_bw_optimize", 0644, NULL, &bw_opt_fops))
		pr_err("proc/enable_bw_optimize create fail\n");
 
	return 0;
}
 
module_init(icc_opt_init);
module_exit(icc_opt_exit);
MODULE_LICENSE("GPL v2");