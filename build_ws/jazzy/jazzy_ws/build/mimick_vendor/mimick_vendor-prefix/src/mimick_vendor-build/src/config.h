#ifndef CONFIG_H_IN_
# define CONFIG_H_IN_

#define MMK_ARCH "x86_64"
#define MMK_BITS 64
#define MMK_MANGLING none

/* #undef MMK_ARCH_x86 */
#define MMK_ARCH_x86_64 1
/* #undef MMK_ARCH_ARM */
/* #undef MMK_ARCH_ARM64 */

#define MMK_EXE_FMT_ELF 1
/* #undef MMK_EXE_FMT_PE */
/* #undef MMK_EXE_FMT_MACH_O */

#define HAVE__R_DEBUG 1
#define HAVE__DYNAMIC 1
#define HAVE_ELF_AUXV_T 1
/* #undef HAVE_ELF_AUXINFO */
#define HAVE_MMAP 1
#define HAVE_MMAP_MAP_ANONYMOUS 1
#define HAVE_MMAP_MAP_ANON 1
/* #undef HAVE___STDIO_COMMON_VFPRINTF */

#endif /* !CONFIG_H_IN_ */
