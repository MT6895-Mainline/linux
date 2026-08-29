/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _MT_PLAT_AEE_H
#define _MT_PLAT_AEE_H

#define DB_OPT_DEFAULT			(0)
#define DB_OPT_MMPROFILE_BUFFER		(1<<17)

static inline void aee_kernel_exception_api_func(const char *file, int line,
						 int db_opt, const char *module,
						 const char *fmt, ...)
{
}

#define aee_kernel_exception_api(file, line, db_opt, module, msg...) \
	aee_kernel_exception_api_func(file, line, db_opt, module, ##msg)

#endif
