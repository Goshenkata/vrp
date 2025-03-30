# 0 "/home/goshenkata/CLionProjects/vrp/main.cpp"
# 1 "/home/goshenkata/CLionProjects/vrp/cmake-build-debug//"
# 0 "<built-in>"
# 0 "<command-line>"
# 1 "/usr/include/stdc-predef.h" 1 3 4
# 0 "<command-line>" 2
# 1 "/home/goshenkata/CLionProjects/vrp/main.cpp"


# 1 "/usr/lib/gcc/x86_64-pc-linux-gnu/14.2.1/include/omp.h" 1 3 4
# 47 "/usr/lib/gcc/x86_64-pc-linux-gnu/14.2.1/include/omp.h" 3 4

# 47 "/usr/lib/gcc/x86_64-pc-linux-gnu/14.2.1/include/omp.h" 3 4
typedef struct
{
  unsigned char _x[4]
    __attribute__((__aligned__(4)));
} omp_lock_t;

typedef struct
{
  unsigned char _x[16]
    __attribute__((__aligned__(8)));
} omp_nest_lock_t;


typedef enum omp_sched_t
{
  omp_sched_static = 1,
  omp_sched_dynamic = 2,
  omp_sched_guided = 3,
  omp_sched_auto = 4,
  omp_sched_monotonic = 0x80000000U
} omp_sched_t;

typedef enum omp_proc_bind_t
{
  omp_proc_bind_false = 0,
  omp_proc_bind_true = 1,
  omp_proc_bind_primary = 2,
  omp_proc_bind_master
    = omp_proc_bind_primary,
  omp_proc_bind_close = 3,
  omp_proc_bind_spread = 4
} omp_proc_bind_t;

typedef enum omp_sync_hint_t
{
  omp_sync_hint_none = 0,
  omp_lock_hint_none = omp_sync_hint_none,
  omp_sync_hint_uncontended = 1,
  omp_lock_hint_uncontended = omp_sync_hint_uncontended,
  omp_sync_hint_contended = 2,
  omp_lock_hint_contended = omp_sync_hint_contended,
  omp_sync_hint_nonspeculative = 4,
  omp_lock_hint_nonspeculative
    = omp_sync_hint_nonspeculative,
  omp_sync_hint_speculative = 8,
  omp_lock_hint_speculative = omp_sync_hint_speculative
} omp_sync_hint_t;

typedef omp_sync_hint_t omp_lock_hint_t;

typedef struct __attribute__((__aligned__ (sizeof (void *)))) omp_depend_t
{
  char __omp_depend_t__[2 * sizeof (void *)];
} omp_depend_t;

typedef enum omp_pause_resource_t
{
  omp_pause_soft = 1,
  omp_pause_hard = 2
} omp_pause_resource_t;

typedef long unsigned int omp_uintptr_t;







typedef enum omp_memspace_handle_t : omp_uintptr_t
{
  omp_default_mem_space = 0,
  omp_large_cap_mem_space = 1,
  omp_const_mem_space = 2,
  omp_high_bw_mem_space = 3,
  omp_low_lat_mem_space = 4,
  __omp_memspace_handle_t_max__ = 0xffffffffffffffffUL
} omp_memspace_handle_t;

typedef enum omp_allocator_handle_t : omp_uintptr_t
{
  omp_null_allocator = 0,
  omp_default_mem_alloc = 1,
  omp_large_cap_mem_alloc = 2,
  omp_const_mem_alloc = 3,
  omp_high_bw_mem_alloc = 4,
  omp_low_lat_mem_alloc = 5,
  omp_cgroup_mem_alloc = 6,
  omp_pteam_mem_alloc = 7,
  omp_thread_mem_alloc = 8,
  __omp_allocator_handle_t_max__ = 0xffffffffffffffffUL
} omp_allocator_handle_t;

typedef enum omp_alloctrait_key_t
{
  omp_atk_sync_hint = 1,
  omp_atk_alignment = 2,
  omp_atk_access = 3,
  omp_atk_pool_size = 4,
  omp_atk_fallback = 5,
  omp_atk_fb_data = 6,
  omp_atk_pinned = 7,
  omp_atk_partition = 8
} omp_alloctrait_key_t;

typedef enum omp_alloctrait_value_t
{
  omp_atv_default = (long unsigned int) -1,
  omp_atv_false = 0,
  omp_atv_true = 1,
  omp_atv_contended = 3,
  omp_atv_uncontended = 4,
  omp_atv_serialized = 5,
  omp_atv_sequential = omp_atv_serialized,
  omp_atv_private = 6,
  omp_atv_all = 7,
  omp_atv_thread = 8,
  omp_atv_pteam = 9,
  omp_atv_cgroup = 10,
  omp_atv_default_mem_fb = 11,
  omp_atv_null_fb = 12,
  omp_atv_abort_fb = 13,
  omp_atv_allocator_fb = 14,
  omp_atv_environment = 15,
  omp_atv_nearest = 16,
  omp_atv_blocked = 17,
  omp_atv_interleaved = 18
} omp_alloctrait_value_t;

typedef struct omp_alloctrait_t
{
  omp_alloctrait_key_t key;
  omp_uintptr_t value;
} omp_alloctrait_t;

typedef enum omp_event_handle_t : omp_uintptr_t
{
  __omp_event_handle_t_max__ = 0xffffffffffffffffUL
} omp_event_handle_t;

enum
{
  omp_initial_device = -1,
  omp_invalid_device = -4
};


extern "C" {







extern void omp_set_num_threads (int) throw ();
extern int omp_get_num_threads (void) throw ();
extern int omp_get_max_threads (void) throw ();
extern int omp_get_thread_num (void) throw ();
extern int omp_get_num_procs (void) throw ();

extern int omp_in_parallel (void) throw ();

extern void omp_set_dynamic (int) throw ();
extern int omp_get_dynamic (void) throw ();

extern void omp_set_nested (int) throw () ;
extern int omp_get_nested (void) throw () ;

extern void omp_init_lock (omp_lock_t *) throw ();
extern void omp_init_lock_with_hint (omp_lock_t *, omp_sync_hint_t)
  throw ();
extern void omp_destroy_lock (omp_lock_t *) throw ();
extern void omp_set_lock (omp_lock_t *) throw ();
extern void omp_unset_lock (omp_lock_t *) throw ();
extern int omp_test_lock (omp_lock_t *) throw ();

extern void omp_init_nest_lock (omp_nest_lock_t *) throw ();
extern void omp_init_nest_lock_with_hint (omp_nest_lock_t *, omp_sync_hint_t)
  throw ();
extern void omp_destroy_nest_lock (omp_nest_lock_t *) throw ();
extern void omp_set_nest_lock (omp_nest_lock_t *) throw ();
extern void omp_unset_nest_lock (omp_nest_lock_t *) throw ();
extern int omp_test_nest_lock (omp_nest_lock_t *) throw ();

extern double omp_get_wtime (void) throw ();
extern double omp_get_wtick (void) throw ();

extern void omp_set_schedule (omp_sched_t, int) throw ();
extern void omp_get_schedule (omp_sched_t *, int *) throw ();
extern int omp_get_thread_limit (void) throw ();
extern void omp_set_max_active_levels (int) throw ();
extern int omp_get_max_active_levels (void) throw ();
extern int omp_get_supported_active_levels (void) throw ();
extern int omp_get_level (void) throw ();
extern int omp_get_ancestor_thread_num (int) throw ();
extern int omp_get_team_size (int) throw ();
extern int omp_get_active_level (void) throw ();

extern int omp_in_final (void) throw ();
extern int omp_in_explicit_task (void) throw ();

extern int omp_get_cancellation (void) throw ();
extern omp_proc_bind_t omp_get_proc_bind (void) throw ();
extern int omp_get_num_places (void) throw ();
extern int omp_get_place_num_procs (int) throw ();
extern void omp_get_place_proc_ids (int, int *) throw ();
extern int omp_get_place_num (void) throw ();
extern int omp_get_partition_num_places (void) throw ();
extern void omp_get_partition_place_nums (int *) throw ();

extern void omp_set_default_device (int) throw ();
extern int omp_get_default_device (void) throw ();
extern int omp_get_num_devices (void) throw ();
extern int omp_get_device_num (void) throw ();
extern int omp_get_num_teams (void) throw ();
extern int omp_get_team_num (void) throw ();

extern int omp_is_initial_device (void) throw ();
extern int omp_get_initial_device (void) throw ();
extern int omp_get_max_task_priority (void) throw ();

extern void omp_fulfill_event (omp_event_handle_t) throw ();

extern void omp_set_num_teams (int) throw ();
extern int omp_get_max_teams (void) throw ();
extern void omp_set_teams_thread_limit (int) throw ();
extern int omp_get_teams_thread_limit (void) throw ();

extern void *omp_target_alloc (long unsigned int, int) throw ();
extern void omp_target_free (void *, int) throw ();
extern int omp_target_is_present (const void *, int) throw ();
extern int omp_target_memcpy (void *, const void *, long unsigned int,
         long unsigned int, long unsigned int, int, int)
  throw ();
extern int omp_target_memcpy_async (void *, const void *, long unsigned int,
        long unsigned int, long unsigned int, int, int,
        int, omp_depend_t *)
  throw ();
extern int omp_target_memcpy_rect (void *, const void *, long unsigned int, int,
       const long unsigned int *,
       const long unsigned int *,
       const long unsigned int *,
       const long unsigned int *,
       const long unsigned int *, int, int)
  throw ();
extern int omp_target_memcpy_rect_async (void *, const void *, long unsigned int,
      int, const long unsigned int *,
      const long unsigned int *,
      const long unsigned int *,
      const long unsigned int *,
      const long unsigned int *, int, int, int,
      omp_depend_t *)
  throw ();
extern int omp_target_associate_ptr (const void *, const void *, long unsigned int,
         long unsigned int, int) throw ();
extern int omp_target_disassociate_ptr (const void *, int) throw ();
extern void *omp_get_mapped_ptr (const void *, int) throw ();
extern int omp_target_is_accessible (const void *, long unsigned int, int)
  throw ();

extern void omp_set_affinity_format (const char *) throw ();
extern long unsigned int omp_get_affinity_format (char *, long unsigned int)
  throw ();
extern void omp_display_affinity (const char *) throw ();
extern long unsigned int omp_capture_affinity (char *, long unsigned int, const char *)
  throw ();

extern int omp_pause_resource (omp_pause_resource_t, int) throw ();
extern int omp_pause_resource_all (omp_pause_resource_t) throw ();

extern omp_allocator_handle_t omp_init_allocator (omp_memspace_handle_t,
        int,
        const omp_alloctrait_t [])
  throw ();
extern void omp_destroy_allocator (omp_allocator_handle_t) throw ();
extern void omp_set_default_allocator (omp_allocator_handle_t) throw ();
extern omp_allocator_handle_t omp_get_default_allocator (void) throw ();
extern void omp_free (void *,
        omp_allocator_handle_t = omp_null_allocator)
  throw ();
extern void *omp_alloc (long unsigned int,
   omp_allocator_handle_t = omp_null_allocator)
  throw () __attribute__((__malloc__, __malloc__ (omp_free),
    __alloc_size__ (1)));
extern void *omp_aligned_alloc (long unsigned int, long unsigned int,
    omp_allocator_handle_t
    = omp_null_allocator)
  throw () __attribute__((__malloc__, __malloc__ (omp_free),
    __alloc_size__ (2), __alloc_align__ (1)));
extern void *omp_calloc (long unsigned int, long unsigned int,
    omp_allocator_handle_t = omp_null_allocator)
  throw () __attribute__((__malloc__, __malloc__ (omp_free),
    __alloc_size__ (1, 2)));
extern void *omp_aligned_calloc (long unsigned int, long unsigned int, long unsigned int,
     omp_allocator_handle_t
     = omp_null_allocator)
  throw () __attribute__((__malloc__, __malloc__ (omp_free),
    __alloc_size__ (2, 3), __alloc_align__ (1)));
extern void *omp_realloc (void *, long unsigned int,
     omp_allocator_handle_t = omp_null_allocator,
     omp_allocator_handle_t = omp_null_allocator)
  throw () __attribute__((__malloc__ (omp_free), __alloc_size__ (2)));

extern void omp_display_env (int) throw ();


}
# 4 "/home/goshenkata/CLionProjects/vrp/main.cpp" 2
# 1 "/usr/include/stdio.h" 1 3 4
# 28 "/usr/include/stdio.h" 3 4
# 1 "/usr/include/bits/libc-header-start.h" 1 3 4
# 33 "/usr/include/bits/libc-header-start.h" 3 4
# 1 "/usr/include/features.h" 1 3 4
# 415 "/usr/include/features.h" 3 4
# 1 "/usr/include/features-time64.h" 1 3 4
# 20 "/usr/include/features-time64.h" 3 4
# 1 "/usr/include/bits/wordsize.h" 1 3 4
# 21 "/usr/include/features-time64.h" 2 3 4
# 1 "/usr/include/bits/timesize.h" 1 3 4
# 19 "/usr/include/bits/timesize.h" 3 4
# 1 "/usr/include/bits/wordsize.h" 1 3 4
# 20 "/usr/include/bits/timesize.h" 2 3 4
# 22 "/usr/include/features-time64.h" 2 3 4
# 416 "/usr/include/features.h" 2 3 4
# 524 "/usr/include/features.h" 3 4
# 1 "/usr/include/sys/cdefs.h" 1 3 4
# 730 "/usr/include/sys/cdefs.h" 3 4
# 1 "/usr/include/bits/wordsize.h" 1 3 4
# 731 "/usr/include/sys/cdefs.h" 2 3 4
# 1 "/usr/include/bits/long-double.h" 1 3 4
# 732 "/usr/include/sys/cdefs.h" 2 3 4
# 525 "/usr/include/features.h" 2 3 4
# 548 "/usr/include/features.h" 3 4
# 1 "/usr/include/gnu/stubs.h" 1 3 4
# 10 "/usr/include/gnu/stubs.h" 3 4
# 1 "/usr/include/gnu/stubs-64.h" 1 3 4
# 11 "/usr/include/gnu/stubs.h" 2 3 4
# 549 "/usr/include/features.h" 2 3 4
# 34 "/usr/include/bits/libc-header-start.h" 2 3 4
# 29 "/usr/include/stdio.h" 2 3 4

extern "C" {



# 1 "/usr/lib/gcc/x86_64-pc-linux-gnu/14.2.1/include/stddef.h" 1 3 4
# 214 "/usr/lib/gcc/x86_64-pc-linux-gnu/14.2.1/include/stddef.h" 3 4
typedef long unsigned int size_t;
# 35 "/usr/include/stdio.h" 2 3 4


# 1 "/usr/lib/gcc/x86_64-pc-linux-gnu/14.2.1/include/stdarg.h" 1 3 4
# 40 "/usr/lib/gcc/x86_64-pc-linux-gnu/14.2.1/include/stdarg.h" 3 4
typedef __builtin_va_list __gnuc_va_list;
# 38 "/usr/include/stdio.h" 2 3 4

# 1 "/usr/include/bits/types.h" 1 3 4
# 27 "/usr/include/bits/types.h" 3 4
# 1 "/usr/include/bits/wordsize.h" 1 3 4
# 28 "/usr/include/bits/types.h" 2 3 4
# 1 "/usr/include/bits/timesize.h" 1 3 4
# 19 "/usr/include/bits/timesize.h" 3 4
# 1 "/usr/include/bits/wordsize.h" 1 3 4
# 20 "/usr/include/bits/timesize.h" 2 3 4
# 29 "/usr/include/bits/types.h" 2 3 4


typedef unsigned char __u_char;
typedef unsigned short int __u_short;
typedef unsigned int __u_int;
typedef unsigned long int __u_long;


typedef signed char __int8_t;
typedef unsigned char __uint8_t;
typedef signed short int __int16_t;
typedef unsigned short int __uint16_t;
typedef signed int __int32_t;
typedef unsigned int __uint32_t;

typedef signed long int __int64_t;
typedef unsigned long int __uint64_t;






typedef __int8_t __int_least8_t;
typedef __uint8_t __uint_least8_t;
typedef __int16_t __int_least16_t;
typedef __uint16_t __uint_least16_t;
typedef __int32_t __int_least32_t;
typedef __uint32_t __uint_least32_t;
typedef __int64_t __int_least64_t;
typedef __uint64_t __uint_least64_t;



typedef long int __quad_t;
typedef unsigned long int __u_quad_t;







typedef long int __intmax_t;
typedef unsigned long int __uintmax_t;
# 141 "/usr/include/bits/types.h" 3 4
# 1 "/usr/include/bits/typesizes.h" 1 3 4
# 142 "/usr/include/bits/types.h" 2 3 4
# 1 "/usr/include/bits/time64.h" 1 3 4
# 143 "/usr/include/bits/types.h" 2 3 4


typedef unsigned long int __dev_t;
typedef unsigned int __uid_t;
typedef unsigned int __gid_t;
typedef unsigned long int __ino_t;
typedef unsigned long int __ino64_t;
typedef unsigned int __mode_t;
typedef unsigned long int __nlink_t;
typedef long int __off_t;
typedef long int __off64_t;
typedef int __pid_t;
typedef struct { int __val[2]; } __fsid_t;
typedef long int __clock_t;
typedef unsigned long int __rlim_t;
typedef unsigned long int __rlim64_t;
typedef unsigned int __id_t;
typedef long int __time_t;
typedef unsigned int __useconds_t;
typedef long int __suseconds_t;
typedef long int __suseconds64_t;

typedef int __daddr_t;
typedef int __key_t;


typedef int __clockid_t;


typedef void * __timer_t;


typedef long int __blksize_t;




typedef long int __blkcnt_t;
typedef long int __blkcnt64_t;


typedef unsigned long int __fsblkcnt_t;
typedef unsigned long int __fsblkcnt64_t;


typedef unsigned long int __fsfilcnt_t;
typedef unsigned long int __fsfilcnt64_t;


typedef long int __fsword_t;

typedef long int __ssize_t;


typedef long int __syscall_slong_t;

typedef unsigned long int __syscall_ulong_t;



typedef __off64_t __loff_t;
typedef char *__caddr_t;


typedef long int __intptr_t;


typedef unsigned int __socklen_t;




typedef int __sig_atomic_t;
# 40 "/usr/include/stdio.h" 2 3 4
# 1 "/usr/include/bits/types/__fpos_t.h" 1 3 4




# 1 "/usr/include/bits/types/__mbstate_t.h" 1 3 4
# 13 "/usr/include/bits/types/__mbstate_t.h" 3 4
typedef struct
{
  int __count;
  union
  {
    unsigned int __wch;
    char __wchb[4];
  } __value;
} __mbstate_t;
# 6 "/usr/include/bits/types/__fpos_t.h" 2 3 4




typedef struct _G_fpos_t
{
  __off_t __pos;
  __mbstate_t __state;
} __fpos_t;
# 41 "/usr/include/stdio.h" 2 3 4
# 1 "/usr/include/bits/types/__fpos64_t.h" 1 3 4
# 10 "/usr/include/bits/types/__fpos64_t.h" 3 4
typedef struct _G_fpos64_t
{
  __off64_t __pos;
  __mbstate_t __state;
} __fpos64_t;
# 42 "/usr/include/stdio.h" 2 3 4
# 1 "/usr/include/bits/types/__FILE.h" 1 3 4



struct _IO_FILE;
typedef struct _IO_FILE __FILE;
# 43 "/usr/include/stdio.h" 2 3 4
# 1 "/usr/include/bits/types/FILE.h" 1 3 4



struct _IO_FILE;


typedef struct _IO_FILE FILE;
# 44 "/usr/include/stdio.h" 2 3 4
# 1 "/usr/include/bits/types/struct_FILE.h" 1 3 4
# 36 "/usr/include/bits/types/struct_FILE.h" 3 4
struct _IO_FILE;
struct _IO_marker;
struct _IO_codecvt;
struct _IO_wide_data;




typedef void _IO_lock_t;





struct _IO_FILE
{
  int _flags;


  char *_IO_read_ptr;
  char *_IO_read_end;
  char *_IO_read_base;
  char *_IO_write_base;
  char *_IO_write_ptr;
  char *_IO_write_end;
  char *_IO_buf_base;
  char *_IO_buf_end;


  char *_IO_save_base;
  char *_IO_backup_base;
  char *_IO_save_end;

  struct _IO_marker *_markers;

  struct _IO_FILE *_chain;

  int _fileno;
  int _flags2:24;

  char _short_backupbuf[1];
  __off_t _old_offset;


  unsigned short _cur_column;
  signed char _vtable_offset;
  char _shortbuf[1];

  _IO_lock_t *_lock;







  __off64_t _offset;

  struct _IO_codecvt *_codecvt;
  struct _IO_wide_data *_wide_data;
  struct _IO_FILE *_freeres_list;
  void *_freeres_buf;
  struct _IO_FILE **_prevchain;
  int _mode;

  char _unused2[15 * sizeof (int) - 5 * sizeof (void *)];
};
# 45 "/usr/include/stdio.h" 2 3 4


# 1 "/usr/include/bits/types/cookie_io_functions_t.h" 1 3 4
# 27 "/usr/include/bits/types/cookie_io_functions_t.h" 3 4
typedef __ssize_t cookie_read_function_t (void *__cookie, char *__buf,
                                          size_t __nbytes);







typedef __ssize_t cookie_write_function_t (void *__cookie, const char *__buf,
                                           size_t __nbytes);







typedef int cookie_seek_function_t (void *__cookie, __off64_t *__pos, int __w);


typedef int cookie_close_function_t (void *__cookie);






typedef struct _IO_cookie_io_functions_t
{
  cookie_read_function_t *read;
  cookie_write_function_t *write;
  cookie_seek_function_t *seek;
  cookie_close_function_t *close;
} cookie_io_functions_t;
# 48 "/usr/include/stdio.h" 2 3 4





typedef __gnuc_va_list va_list;
# 64 "/usr/include/stdio.h" 3 4
typedef __off_t off_t;






typedef __off64_t off64_t;






typedef __ssize_t ssize_t;






typedef __fpos_t fpos_t;




typedef __fpos64_t fpos64_t;
# 129 "/usr/include/stdio.h" 3 4
# 1 "/usr/include/bits/stdio_lim.h" 1 3 4
# 130 "/usr/include/stdio.h" 2 3 4
# 149 "/usr/include/stdio.h" 3 4
extern FILE *stdin;
extern FILE *stdout;
extern FILE *stderr;






extern int remove (const char *__filename) noexcept (true);

extern int rename (const char *__old, const char *__new) noexcept (true);



extern int renameat (int __oldfd, const char *__old, int __newfd,
       const char *__new) noexcept (true);
# 176 "/usr/include/stdio.h" 3 4
extern int renameat2 (int __oldfd, const char *__old, int __newfd,
        const char *__new, unsigned int __flags) noexcept (true);






extern int fclose (FILE *__stream) __attribute__ ((__nonnull__ (1)));
# 194 "/usr/include/stdio.h" 3 4
extern FILE *tmpfile (void)
  __attribute__ ((__malloc__)) __attribute__ ((__malloc__ (fclose, 1))) ;
# 206 "/usr/include/stdio.h" 3 4
extern FILE *tmpfile64 (void)
   __attribute__ ((__malloc__)) __attribute__ ((__malloc__ (fclose, 1))) ;



extern char *tmpnam (char[20]) noexcept (true) ;




extern char *tmpnam_r (char __s[20]) noexcept (true) ;
# 228 "/usr/include/stdio.h" 3 4
extern char *tempnam (const char *__dir, const char *__pfx)
   noexcept (true) __attribute__ ((__malloc__)) __attribute__ ((__malloc__ (__builtin_free, 1)));






extern int fflush (FILE *__stream);
# 245 "/usr/include/stdio.h" 3 4
extern int fflush_unlocked (FILE *__stream);
# 255 "/usr/include/stdio.h" 3 4
extern int fcloseall (void);
# 264 "/usr/include/stdio.h" 3 4
extern FILE *fopen (const char *__restrict __filename,
      const char *__restrict __modes)
  __attribute__ ((__malloc__)) __attribute__ ((__malloc__ (fclose, 1))) ;




extern FILE *freopen (const char *__restrict __filename,
        const char *__restrict __modes,
        FILE *__restrict __stream) __attribute__ ((__nonnull__ (3)));
# 289 "/usr/include/stdio.h" 3 4
extern FILE *fopen64 (const char *__restrict __filename,
        const char *__restrict __modes)
  __attribute__ ((__malloc__)) __attribute__ ((__malloc__ (fclose, 1))) ;
extern FILE *freopen64 (const char *__restrict __filename,
   const char *__restrict __modes,
   FILE *__restrict __stream) __attribute__ ((__nonnull__ (3)));




extern FILE *fdopen (int __fd, const char *__modes) noexcept (true)
  __attribute__ ((__malloc__)) __attribute__ ((__malloc__ (fclose, 1))) ;





extern FILE *fopencookie (void *__restrict __magic_cookie,
     const char *__restrict __modes,
     cookie_io_functions_t __io_funcs) noexcept (true)
  __attribute__ ((__malloc__)) __attribute__ ((__malloc__ (fclose, 1))) ;




extern FILE *fmemopen (void *__s, size_t __len, const char *__modes)
  noexcept (true) __attribute__ ((__malloc__)) __attribute__ ((__malloc__ (fclose, 1))) ;




extern FILE *open_memstream (char **__bufloc, size_t *__sizeloc) noexcept (true)
  __attribute__ ((__malloc__)) __attribute__ ((__malloc__ (fclose, 1))) ;
# 334 "/usr/include/stdio.h" 3 4
extern void setbuf (FILE *__restrict __stream, char *__restrict __buf) noexcept (true)
  __attribute__ ((__nonnull__ (1)));



extern int setvbuf (FILE *__restrict __stream, char *__restrict __buf,
      int __modes, size_t __n) noexcept (true) __attribute__ ((__nonnull__ (1)));




extern void setbuffer (FILE *__restrict __stream, char *__restrict __buf,
         size_t __size) noexcept (true) __attribute__ ((__nonnull__ (1)));


extern void setlinebuf (FILE *__stream) noexcept (true) __attribute__ ((__nonnull__ (1)));







extern int fprintf (FILE *__restrict __stream,
      const char *__restrict __format, ...) __attribute__ ((__nonnull__ (1)));




extern int printf (const char *__restrict __format, ...);

extern int sprintf (char *__restrict __s,
      const char *__restrict __format, ...) noexcept (true);





extern int vfprintf (FILE *__restrict __s, const char *__restrict __format,
       __gnuc_va_list __arg) __attribute__ ((__nonnull__ (1)));




extern int vprintf (const char *__restrict __format, __gnuc_va_list __arg);

extern int vsprintf (char *__restrict __s, const char *__restrict __format,
       __gnuc_va_list __arg) noexcept (true);



extern int snprintf (char *__restrict __s, size_t __maxlen,
       const char *__restrict __format, ...)
     noexcept (true) __attribute__ ((__format__ (__printf__, 3, 4)));

extern int vsnprintf (char *__restrict __s, size_t __maxlen,
        const char *__restrict __format, __gnuc_va_list __arg)
     noexcept (true) __attribute__ ((__format__ (__printf__, 3, 0)));





extern int vasprintf (char **__restrict __ptr, const char *__restrict __f,
        __gnuc_va_list __arg)
     noexcept (true) __attribute__ ((__format__ (__printf__, 2, 0))) ;
extern int __asprintf (char **__restrict __ptr,
         const char *__restrict __fmt, ...)
     noexcept (true) __attribute__ ((__format__ (__printf__, 2, 3))) ;
extern int asprintf (char **__restrict __ptr,
       const char *__restrict __fmt, ...)
     noexcept (true) __attribute__ ((__format__ (__printf__, 2, 3))) ;




extern int vdprintf (int __fd, const char *__restrict __fmt,
       __gnuc_va_list __arg)
     __attribute__ ((__format__ (__printf__, 2, 0)));
extern int dprintf (int __fd, const char *__restrict __fmt, ...)
     __attribute__ ((__format__ (__printf__, 2, 3)));







extern int fscanf (FILE *__restrict __stream,
     const char *__restrict __format, ...) __attribute__ ((__nonnull__ (1)));




extern int scanf (const char *__restrict __format, ...) ;

extern int sscanf (const char *__restrict __s,
     const char *__restrict __format, ...) noexcept (true);





# 1 "/usr/include/bits/floatn.h" 1 3 4
# 131 "/usr/include/bits/floatn.h" 3 4
# 1 "/usr/include/bits/floatn-common.h" 1 3 4
# 24 "/usr/include/bits/floatn-common.h" 3 4
# 1 "/usr/include/bits/long-double.h" 1 3 4
# 25 "/usr/include/bits/floatn-common.h" 2 3 4
# 132 "/usr/include/bits/floatn.h" 2 3 4
# 438 "/usr/include/stdio.h" 2 3 4




extern int fscanf (FILE *__restrict __stream, const char *__restrict __format, ...) __asm__ ("" "__isoc23_fscanf")

                                __attribute__ ((__nonnull__ (1)));
extern int scanf (const char *__restrict __format, ...) __asm__ ("" "__isoc23_scanf")
                              ;
extern int sscanf (const char *__restrict __s, const char *__restrict __format, ...) noexcept (true) __asm__ ("" "__isoc23_sscanf")

                      ;
# 490 "/usr/include/stdio.h" 3 4
extern int vfscanf (FILE *__restrict __s, const char *__restrict __format,
      __gnuc_va_list __arg)
     __attribute__ ((__format__ (__scanf__, 2, 0))) __attribute__ ((__nonnull__ (1)));





extern int vscanf (const char *__restrict __format, __gnuc_va_list __arg)
     __attribute__ ((__format__ (__scanf__, 1, 0))) ;


extern int vsscanf (const char *__restrict __s,
      const char *__restrict __format, __gnuc_va_list __arg)
     noexcept (true) __attribute__ ((__format__ (__scanf__, 2, 0)));






extern int vfscanf (FILE *__restrict __s, const char *__restrict __format, __gnuc_va_list __arg) __asm__ ("" "__isoc23_vfscanf")



     __attribute__ ((__format__ (__scanf__, 2, 0))) __attribute__ ((__nonnull__ (1)));
extern int vscanf (const char *__restrict __format, __gnuc_va_list __arg) __asm__ ("" "__isoc23_vscanf")

     __attribute__ ((__format__ (__scanf__, 1, 0))) ;
extern int vsscanf (const char *__restrict __s, const char *__restrict __format, __gnuc_va_list __arg) noexcept (true) __asm__ ("" "__isoc23_vsscanf")



     __attribute__ ((__format__ (__scanf__, 2, 0)));
# 575 "/usr/include/stdio.h" 3 4
extern int fgetc (FILE *__stream) __attribute__ ((__nonnull__ (1)));
extern int getc (FILE *__stream) __attribute__ ((__nonnull__ (1)));





extern int getchar (void);






extern int getc_unlocked (FILE *__stream) __attribute__ ((__nonnull__ (1)));
extern int getchar_unlocked (void);
# 600 "/usr/include/stdio.h" 3 4
extern int fgetc_unlocked (FILE *__stream) __attribute__ ((__nonnull__ (1)));
# 611 "/usr/include/stdio.h" 3 4
extern int fputc (int __c, FILE *__stream) __attribute__ ((__nonnull__ (2)));
extern int putc (int __c, FILE *__stream) __attribute__ ((__nonnull__ (2)));





extern int putchar (int __c);
# 627 "/usr/include/stdio.h" 3 4
extern int fputc_unlocked (int __c, FILE *__stream) __attribute__ ((__nonnull__ (2)));







extern int putc_unlocked (int __c, FILE *__stream) __attribute__ ((__nonnull__ (2)));
extern int putchar_unlocked (int __c);






extern int getw (FILE *__stream) __attribute__ ((__nonnull__ (1)));


extern int putw (int __w, FILE *__stream) __attribute__ ((__nonnull__ (2)));







extern char *fgets (char *__restrict __s, int __n, FILE *__restrict __stream)
     __attribute__ ((__access__ (__write_only__, 1, 2))) __attribute__ ((__nonnull__ (3)));
# 677 "/usr/include/stdio.h" 3 4
extern char *fgets_unlocked (char *__restrict __s, int __n,
        FILE *__restrict __stream)
    __attribute__ ((__access__ (__write_only__, 1, 2))) __attribute__ ((__nonnull__ (3)));
# 689 "/usr/include/stdio.h" 3 4
extern __ssize_t __getdelim (char **__restrict __lineptr,
                             size_t *__restrict __n, int __delimiter,
                             FILE *__restrict __stream) __attribute__ ((__nonnull__ (4)));
extern __ssize_t getdelim (char **__restrict __lineptr,
                           size_t *__restrict __n, int __delimiter,
                           FILE *__restrict __stream) __attribute__ ((__nonnull__ (4)));


extern __ssize_t getline (char **__restrict __lineptr,
                          size_t *__restrict __n,
                          FILE *__restrict __stream) __attribute__ ((__nonnull__ (3)));







extern int fputs (const char *__restrict __s, FILE *__restrict __stream)
  __attribute__ ((__nonnull__ (2)));





extern int puts (const char *__s);






extern int ungetc (int __c, FILE *__stream) __attribute__ ((__nonnull__ (2)));






extern size_t fread (void *__restrict __ptr, size_t __size,
       size_t __n, FILE *__restrict __stream)
  __attribute__ ((__nonnull__ (4)));




extern size_t fwrite (const void *__restrict __ptr, size_t __size,
        size_t __n, FILE *__restrict __s) __attribute__ ((__nonnull__ (4)));
# 745 "/usr/include/stdio.h" 3 4
extern int fputs_unlocked (const char *__restrict __s,
      FILE *__restrict __stream) __attribute__ ((__nonnull__ (2)));
# 756 "/usr/include/stdio.h" 3 4
extern size_t fread_unlocked (void *__restrict __ptr, size_t __size,
         size_t __n, FILE *__restrict __stream)
  __attribute__ ((__nonnull__ (4)));
extern size_t fwrite_unlocked (const void *__restrict __ptr, size_t __size,
          size_t __n, FILE *__restrict __stream)
  __attribute__ ((__nonnull__ (4)));







extern int fseek (FILE *__stream, long int __off, int __whence)
  __attribute__ ((__nonnull__ (1)));




extern long int ftell (FILE *__stream) __attribute__ ((__nonnull__ (1)));




extern void rewind (FILE *__stream) __attribute__ ((__nonnull__ (1)));
# 793 "/usr/include/stdio.h" 3 4
extern int fseeko (FILE *__stream, __off_t __off, int __whence)
  __attribute__ ((__nonnull__ (1)));




extern __off_t ftello (FILE *__stream) __attribute__ ((__nonnull__ (1)));
# 819 "/usr/include/stdio.h" 3 4
extern int fgetpos (FILE *__restrict __stream, fpos_t *__restrict __pos)
  __attribute__ ((__nonnull__ (1)));




extern int fsetpos (FILE *__stream, const fpos_t *__pos) __attribute__ ((__nonnull__ (1)));
# 841 "/usr/include/stdio.h" 3 4
extern int fseeko64 (FILE *__stream, __off64_t __off, int __whence)
  __attribute__ ((__nonnull__ (1)));
extern __off64_t ftello64 (FILE *__stream) __attribute__ ((__nonnull__ (1)));
extern int fgetpos64 (FILE *__restrict __stream, fpos64_t *__restrict __pos)
  __attribute__ ((__nonnull__ (1)));
extern int fsetpos64 (FILE *__stream, const fpos64_t *__pos) __attribute__ ((__nonnull__ (1)));



extern void clearerr (FILE *__stream) noexcept (true) __attribute__ ((__nonnull__ (1)));

extern int feof (FILE *__stream) noexcept (true) __attribute__ ((__nonnull__ (1)));

extern int ferror (FILE *__stream) noexcept (true) __attribute__ ((__nonnull__ (1)));



extern void clearerr_unlocked (FILE *__stream) noexcept (true) __attribute__ ((__nonnull__ (1)));
extern int feof_unlocked (FILE *__stream) noexcept (true) __attribute__ ((__nonnull__ (1)));
extern int ferror_unlocked (FILE *__stream) noexcept (true) __attribute__ ((__nonnull__ (1)));







extern void perror (const char *__s) __attribute__ ((__cold__));




extern int fileno (FILE *__stream) noexcept (true) __attribute__ ((__nonnull__ (1)));




extern int fileno_unlocked (FILE *__stream) noexcept (true) __attribute__ ((__nonnull__ (1)));
# 887 "/usr/include/stdio.h" 3 4
extern int pclose (FILE *__stream) __attribute__ ((__nonnull__ (1)));





extern FILE *popen (const char *__command, const char *__modes)
  __attribute__ ((__malloc__)) __attribute__ ((__malloc__ (pclose, 1))) ;






extern char *ctermid (char *__s) noexcept (true)
  __attribute__ ((__access__ (__write_only__, 1)));





extern char *cuserid (char *__s)
  __attribute__ ((__access__ (__write_only__, 1)));




struct obstack;


extern int obstack_printf (struct obstack *__restrict __obstack,
      const char *__restrict __format, ...)
     noexcept (true) __attribute__ ((__format__ (__printf__, 2, 3)));
extern int obstack_vprintf (struct obstack *__restrict __obstack,
       const char *__restrict __format,
       __gnuc_va_list __args)
     noexcept (true) __attribute__ ((__format__ (__printf__, 2, 0)));







extern void flockfile (FILE *__stream) noexcept (true) __attribute__ ((__nonnull__ (1)));



extern int ftrylockfile (FILE *__stream) noexcept (true) __attribute__ ((__nonnull__ (1)));


extern void funlockfile (FILE *__stream) noexcept (true) __attribute__ ((__nonnull__ (1)));
# 949 "/usr/include/stdio.h" 3 4
extern int __uflow (FILE *);
extern int __overflow (FILE *, int);
# 973 "/usr/include/stdio.h" 3 4
}
# 5 "/home/goshenkata/CLionProjects/vrp/main.cpp" 2
# 1 "/usr/include/c++/14.2.1/stdlib.h" 1 3
# 36 "/usr/include/c++/14.2.1/stdlib.h" 3
# 1 "/usr/include/c++/14.2.1/cstdlib" 1 3
# 39 "/usr/include/c++/14.2.1/cstdlib" 3
       
# 40 "/usr/include/c++/14.2.1/cstdlib" 3

# 1 "/usr/include/c++/14.2.1/x86_64-pc-linux-gnu/bits/c++config.h" 1 3
# 33 "/usr/include/c++/14.2.1/x86_64-pc-linux-gnu/bits/c++config.h" 3
       
# 34 "/usr/include/c++/14.2.1/x86_64-pc-linux-gnu/bits/c++config.h" 3
# 308 "/usr/include/c++/14.2.1/x86_64-pc-linux-gnu/bits/c++config.h" 3
namespace std
{
  typedef long unsigned int size_t;
  typedef long int ptrdiff_t;


  typedef decltype(nullptr) nullptr_t;


#pragma GCC visibility push(default)


  extern "C++" __attribute__ ((__noreturn__, __always_inline__))
  inline void __terminate() noexcept
  {
    void terminate() noexcept __attribute__ ((__noreturn__,__cold__));
    terminate();
  }
#pragma GCC visibility pop
}
# 341 "/usr/include/c++/14.2.1/x86_64-pc-linux-gnu/bits/c++config.h" 3
namespace std
{
  inline namespace __cxx11 __attribute__((__abi_tag__ ("cxx11"))) { }
}
namespace __gnu_cxx
{
  inline namespace __cxx11 __attribute__((__abi_tag__ ("cxx11"))) { }
}
# 534 "/usr/include/c++/14.2.1/x86_64-pc-linux-gnu/bits/c++config.h" 3
namespace std
{
#pragma GCC visibility push(default)




  __attribute__((__always_inline__))
  constexpr inline bool
  __is_constant_evaluated() noexcept
  {





    return __builtin_is_constant_evaluated();



  }
#pragma GCC visibility pop
}
# 573 "/usr/include/c++/14.2.1/x86_64-pc-linux-gnu/bits/c++config.h" 3
namespace std
{
#pragma GCC visibility push(default)

  extern "C++" __attribute__ ((__noreturn__))
  void
  __glibcxx_assert_fail
    (const char* __file, int __line, const char* __function,
     const char* __condition)
  noexcept;
#pragma GCC visibility pop
}
# 604 "/usr/include/c++/14.2.1/x86_64-pc-linux-gnu/bits/c++config.h" 3
namespace std
{
  __attribute__((__always_inline__,__visibility__("default")))
  inline void
  __glibcxx_assert_fail()
  { }
}
# 683 "/usr/include/c++/14.2.1/x86_64-pc-linux-gnu/bits/c++config.h" 3
# 1 "/usr/include/c++/14.2.1/x86_64-pc-linux-gnu/bits/os_defines.h" 1 3
# 684 "/usr/include/c++/14.2.1/x86_64-pc-linux-gnu/bits/c++config.h" 2 3


# 1 "/usr/include/c++/14.2.1/x86_64-pc-linux-gnu/bits/cpu_defines.h" 1 3
# 687 "/usr/include/c++/14.2.1/x86_64-pc-linux-gnu/bits/c++config.h" 2 3
# 828 "/usr/include/c++/14.2.1/x86_64-pc-linux-gnu/bits/c++config.h" 3
namespace __gnu_cxx
{
  typedef __decltype(0.0bf16) __bfloat16_t;
}
# 890 "/usr/include/c++/14.2.1/x86_64-pc-linux-gnu/bits/c++config.h" 3
# 1 "/usr/include/c++/14.2.1/pstl/pstl_config.h" 1 3
# 891 "/usr/include/c++/14.2.1/x86_64-pc-linux-gnu/bits/c++config.h" 2 3
# 42 "/usr/include/c++/14.2.1/cstdlib" 2 3
# 79 "/usr/include/c++/14.2.1/cstdlib" 3
# 1 "/usr/include/stdlib.h" 1 3 4
# 26 "/usr/include/stdlib.h" 3 4
# 1 "/usr/include/bits/libc-header-start.h" 1 3 4
# 27 "/usr/include/stdlib.h" 2 3 4





# 1 "/usr/lib/gcc/x86_64-pc-linux-gnu/14.2.1/include/stddef.h" 1 3 4
# 33 "/usr/include/stdlib.h" 2 3 4

extern "C" {





# 1 "/usr/include/bits/waitflags.h" 1 3 4
# 41 "/usr/include/stdlib.h" 2 3 4
# 1 "/usr/include/bits/waitstatus.h" 1 3 4
# 42 "/usr/include/stdlib.h" 2 3 4
# 59 "/usr/include/stdlib.h" 3 4
typedef struct
  {
    int quot;
    int rem;
  } div_t;



typedef struct
  {
    long int quot;
    long int rem;
  } ldiv_t;





__extension__ typedef struct
  {
    long long int quot;
    long long int rem;
  } lldiv_t;
# 98 "/usr/include/stdlib.h" 3 4
extern size_t __ctype_get_mb_cur_max (void) noexcept (true) ;



extern double atof (const char *__nptr)
     noexcept (true) __attribute__ ((__pure__)) __attribute__ ((__nonnull__ (1))) ;

extern int atoi (const char *__nptr)
     noexcept (true) __attribute__ ((__pure__)) __attribute__ ((__nonnull__ (1))) ;

extern long int atol (const char *__nptr)
     noexcept (true) __attribute__ ((__pure__)) __attribute__ ((__nonnull__ (1))) ;



__extension__ extern long long int atoll (const char *__nptr)
     noexcept (true) __attribute__ ((__pure__)) __attribute__ ((__nonnull__ (1))) ;



extern double strtod (const char *__restrict __nptr,
        char **__restrict __endptr)
     noexcept (true) __attribute__ ((__nonnull__ (1)));



extern float strtof (const char *__restrict __nptr,
       char **__restrict __endptr) noexcept (true) __attribute__ ((__nonnull__ (1)));

extern long double strtold (const char *__restrict __nptr,
       char **__restrict __endptr)
     noexcept (true) __attribute__ ((__nonnull__ (1)));
# 141 "/usr/include/stdlib.h" 3 4
extern _Float32 strtof32 (const char *__restrict __nptr,
     char **__restrict __endptr)
     noexcept (true) __attribute__ ((__nonnull__ (1)));



extern _Float64 strtof64 (const char *__restrict __nptr,
     char **__restrict __endptr)
     noexcept (true) __attribute__ ((__nonnull__ (1)));



extern _Float128 strtof128 (const char *__restrict __nptr,
       char **__restrict __endptr)
     noexcept (true) __attribute__ ((__nonnull__ (1)));



extern _Float32x strtof32x (const char *__restrict __nptr,
       char **__restrict __endptr)
     noexcept (true) __attribute__ ((__nonnull__ (1)));



extern _Float64x strtof64x (const char *__restrict __nptr,
       char **__restrict __endptr)
     noexcept (true) __attribute__ ((__nonnull__ (1)));
# 177 "/usr/include/stdlib.h" 3 4
extern long int strtol (const char *__restrict __nptr,
   char **__restrict __endptr, int __base)
     noexcept (true) __attribute__ ((__nonnull__ (1)));

extern unsigned long int strtoul (const char *__restrict __nptr,
      char **__restrict __endptr, int __base)
     noexcept (true) __attribute__ ((__nonnull__ (1)));



__extension__
extern long long int strtoq (const char *__restrict __nptr,
        char **__restrict __endptr, int __base)
     noexcept (true) __attribute__ ((__nonnull__ (1)));

__extension__
extern unsigned long long int strtouq (const char *__restrict __nptr,
           char **__restrict __endptr, int __base)
     noexcept (true) __attribute__ ((__nonnull__ (1)));




__extension__
extern long long int strtoll (const char *__restrict __nptr,
         char **__restrict __endptr, int __base)
     noexcept (true) __attribute__ ((__nonnull__ (1)));

__extension__
extern unsigned long long int strtoull (const char *__restrict __nptr,
     char **__restrict __endptr, int __base)
     noexcept (true) __attribute__ ((__nonnull__ (1)));






extern long int strtol (const char *__restrict __nptr, char **__restrict __endptr, int __base) noexcept (true) __asm__ ("" "__isoc23_strtol")


     __attribute__ ((__nonnull__ (1)));
extern unsigned long int strtoul (const char *__restrict __nptr, char **__restrict __endptr, int __base) noexcept (true) __asm__ ("" "__isoc23_strtoul")



     __attribute__ ((__nonnull__ (1)));

__extension__
extern long long int strtoq (const char *__restrict __nptr, char **__restrict __endptr, int __base) noexcept (true) __asm__ ("" "__isoc23_strtoll")


     __attribute__ ((__nonnull__ (1)));
__extension__
extern unsigned long long int strtouq (const char *__restrict __nptr, char **__restrict __endptr, int __base) noexcept (true) __asm__ ("" "__isoc23_strtoull")



     __attribute__ ((__nonnull__ (1)));

__extension__
extern long long int strtoll (const char *__restrict __nptr, char **__restrict __endptr, int __base) noexcept (true) __asm__ ("" "__isoc23_strtoll")


     __attribute__ ((__nonnull__ (1)));
__extension__
extern unsigned long long int strtoull (const char *__restrict __nptr, char **__restrict __endptr, int __base) noexcept (true) __asm__ ("" "__isoc23_strtoull")



     __attribute__ ((__nonnull__ (1)));
# 278 "/usr/include/stdlib.h" 3 4
extern int strfromd (char *__dest, size_t __size, const char *__format,
       double __f)
     noexcept (true) __attribute__ ((__nonnull__ (3)));

extern int strfromf (char *__dest, size_t __size, const char *__format,
       float __f)
     noexcept (true) __attribute__ ((__nonnull__ (3)));

extern int strfroml (char *__dest, size_t __size, const char *__format,
       long double __f)
     noexcept (true) __attribute__ ((__nonnull__ (3)));
# 298 "/usr/include/stdlib.h" 3 4
extern int strfromf32 (char *__dest, size_t __size, const char * __format,
         _Float32 __f)
     noexcept (true) __attribute__ ((__nonnull__ (3)));



extern int strfromf64 (char *__dest, size_t __size, const char * __format,
         _Float64 __f)
     noexcept (true) __attribute__ ((__nonnull__ (3)));



extern int strfromf128 (char *__dest, size_t __size, const char * __format,
   _Float128 __f)
     noexcept (true) __attribute__ ((__nonnull__ (3)));



extern int strfromf32x (char *__dest, size_t __size, const char * __format,
   _Float32x __f)
     noexcept (true) __attribute__ ((__nonnull__ (3)));



extern int strfromf64x (char *__dest, size_t __size, const char * __format,
   _Float64x __f)
     noexcept (true) __attribute__ ((__nonnull__ (3)));
# 338 "/usr/include/stdlib.h" 3 4
# 1 "/usr/include/bits/types/locale_t.h" 1 3 4
# 22 "/usr/include/bits/types/locale_t.h" 3 4
# 1 "/usr/include/bits/types/__locale_t.h" 1 3 4
# 27 "/usr/include/bits/types/__locale_t.h" 3 4
struct __locale_struct
{

  struct __locale_data *__locales[13];


  const unsigned short int *__ctype_b;
  const int *__ctype_tolower;
  const int *__ctype_toupper;


  const char *__names[13];
};

typedef struct __locale_struct *__locale_t;
# 23 "/usr/include/bits/types/locale_t.h" 2 3 4

typedef __locale_t locale_t;
# 339 "/usr/include/stdlib.h" 2 3 4

extern long int strtol_l (const char *__restrict __nptr,
     char **__restrict __endptr, int __base,
     locale_t __loc) noexcept (true) __attribute__ ((__nonnull__ (1, 4)));

extern unsigned long int strtoul_l (const char *__restrict __nptr,
        char **__restrict __endptr,
        int __base, locale_t __loc)
     noexcept (true) __attribute__ ((__nonnull__ (1, 4)));

__extension__
extern long long int strtoll_l (const char *__restrict __nptr,
    char **__restrict __endptr, int __base,
    locale_t __loc)
     noexcept (true) __attribute__ ((__nonnull__ (1, 4)));

__extension__
extern unsigned long long int strtoull_l (const char *__restrict __nptr,
       char **__restrict __endptr,
       int __base, locale_t __loc)
     noexcept (true) __attribute__ ((__nonnull__ (1, 4)));





extern long int strtol_l (const char *__restrict __nptr, char **__restrict __endptr, int __base, locale_t __loc) noexcept (true) __asm__ ("" "__isoc23_strtol_l")



     __attribute__ ((__nonnull__ (1, 4)));
extern unsigned long int strtoul_l (const char *__restrict __nptr, char **__restrict __endptr, int __base, locale_t __loc) noexcept (true) __asm__ ("" "__isoc23_strtoul_l")




     __attribute__ ((__nonnull__ (1, 4)));
__extension__
extern long long int strtoll_l (const char *__restrict __nptr, char **__restrict __endptr, int __base, locale_t __loc) noexcept (true) __asm__ ("" "__isoc23_strtoll_l")




     __attribute__ ((__nonnull__ (1, 4)));
__extension__
extern unsigned long long int strtoull_l (const char *__restrict __nptr, char **__restrict __endptr, int __base, locale_t __loc) noexcept (true) __asm__ ("" "__isoc23_strtoull_l")




     __attribute__ ((__nonnull__ (1, 4)));
# 415 "/usr/include/stdlib.h" 3 4
extern double strtod_l (const char *__restrict __nptr,
   char **__restrict __endptr, locale_t __loc)
     noexcept (true) __attribute__ ((__nonnull__ (1, 3)));

extern float strtof_l (const char *__restrict __nptr,
         char **__restrict __endptr, locale_t __loc)
     noexcept (true) __attribute__ ((__nonnull__ (1, 3)));

extern long double strtold_l (const char *__restrict __nptr,
         char **__restrict __endptr,
         locale_t __loc)
     noexcept (true) __attribute__ ((__nonnull__ (1, 3)));
# 436 "/usr/include/stdlib.h" 3 4
extern _Float32 strtof32_l (const char *__restrict __nptr,
       char **__restrict __endptr,
       locale_t __loc)
     noexcept (true) __attribute__ ((__nonnull__ (1, 3)));



extern _Float64 strtof64_l (const char *__restrict __nptr,
       char **__restrict __endptr,
       locale_t __loc)
     noexcept (true) __attribute__ ((__nonnull__ (1, 3)));



extern _Float128 strtof128_l (const char *__restrict __nptr,
         char **__restrict __endptr,
         locale_t __loc)
     noexcept (true) __attribute__ ((__nonnull__ (1, 3)));



extern _Float32x strtof32x_l (const char *__restrict __nptr,
         char **__restrict __endptr,
         locale_t __loc)
     noexcept (true) __attribute__ ((__nonnull__ (1, 3)));



extern _Float64x strtof64x_l (const char *__restrict __nptr,
         char **__restrict __endptr,
         locale_t __loc)
     noexcept (true) __attribute__ ((__nonnull__ (1, 3)));
# 505 "/usr/include/stdlib.h" 3 4
extern char *l64a (long int __n) noexcept (true) ;


extern long int a64l (const char *__s)
     noexcept (true) __attribute__ ((__pure__)) __attribute__ ((__nonnull__ (1))) ;




# 1 "/usr/include/sys/types.h" 1 3 4
# 27 "/usr/include/sys/types.h" 3 4
extern "C" {





typedef __u_char u_char;
typedef __u_short u_short;
typedef __u_int u_int;
typedef __u_long u_long;
typedef __quad_t quad_t;
typedef __u_quad_t u_quad_t;
typedef __fsid_t fsid_t;


typedef __loff_t loff_t;




typedef __ino_t ino_t;






typedef __ino64_t ino64_t;




typedef __dev_t dev_t;




typedef __gid_t gid_t;




typedef __mode_t mode_t;




typedef __nlink_t nlink_t;




typedef __uid_t uid_t;
# 97 "/usr/include/sys/types.h" 3 4
typedef __pid_t pid_t;





typedef __id_t id_t;
# 114 "/usr/include/sys/types.h" 3 4
typedef __daddr_t daddr_t;
typedef __caddr_t caddr_t;





typedef __key_t key_t;




# 1 "/usr/include/bits/types/clock_t.h" 1 3 4






typedef __clock_t clock_t;
# 127 "/usr/include/sys/types.h" 2 3 4

# 1 "/usr/include/bits/types/clockid_t.h" 1 3 4






typedef __clockid_t clockid_t;
# 129 "/usr/include/sys/types.h" 2 3 4
# 1 "/usr/include/bits/types/time_t.h" 1 3 4
# 10 "/usr/include/bits/types/time_t.h" 3 4
typedef __time_t time_t;
# 130 "/usr/include/sys/types.h" 2 3 4
# 1 "/usr/include/bits/types/timer_t.h" 1 3 4






typedef __timer_t timer_t;
# 131 "/usr/include/sys/types.h" 2 3 4



typedef __useconds_t useconds_t;



typedef __suseconds_t suseconds_t;





# 1 "/usr/lib/gcc/x86_64-pc-linux-gnu/14.2.1/include/stddef.h" 1 3 4
# 145 "/usr/include/sys/types.h" 2 3 4



typedef unsigned long int ulong;
typedef unsigned short int ushort;
typedef unsigned int uint;




# 1 "/usr/include/bits/stdint-intn.h" 1 3 4
# 24 "/usr/include/bits/stdint-intn.h" 3 4
typedef __int8_t int8_t;
typedef __int16_t int16_t;
typedef __int32_t int32_t;
typedef __int64_t int64_t;
# 156 "/usr/include/sys/types.h" 2 3 4


typedef __uint8_t u_int8_t;
typedef __uint16_t u_int16_t;
typedef __uint32_t u_int32_t;
typedef __uint64_t u_int64_t;


typedef int register_t __attribute__ ((__mode__ (__word__)));
# 176 "/usr/include/sys/types.h" 3 4
# 1 "/usr/include/endian.h" 1 3 4
# 24 "/usr/include/endian.h" 3 4
# 1 "/usr/include/bits/endian.h" 1 3 4
# 35 "/usr/include/bits/endian.h" 3 4
# 1 "/usr/include/bits/endianness.h" 1 3 4
# 36 "/usr/include/bits/endian.h" 2 3 4
# 25 "/usr/include/endian.h" 2 3 4
# 35 "/usr/include/endian.h" 3 4
# 1 "/usr/include/bits/byteswap.h" 1 3 4
# 33 "/usr/include/bits/byteswap.h" 3 4
static __inline __uint16_t
__bswap_16 (__uint16_t __bsx)
{

  return __builtin_bswap16 (__bsx);



}






static __inline __uint32_t
__bswap_32 (__uint32_t __bsx)
{

  return __builtin_bswap32 (__bsx);



}
# 69 "/usr/include/bits/byteswap.h" 3 4
__extension__ static __inline __uint64_t
__bswap_64 (__uint64_t __bsx)
{

  return __builtin_bswap64 (__bsx);



}
# 36 "/usr/include/endian.h" 2 3 4
# 1 "/usr/include/bits/uintn-identity.h" 1 3 4
# 32 "/usr/include/bits/uintn-identity.h" 3 4
static __inline __uint16_t
__uint16_identity (__uint16_t __x)
{
  return __x;
}

static __inline __uint32_t
__uint32_identity (__uint32_t __x)
{
  return __x;
}

static __inline __uint64_t
__uint64_identity (__uint64_t __x)
{
  return __x;
}
# 37 "/usr/include/endian.h" 2 3 4
# 177 "/usr/include/sys/types.h" 2 3 4


# 1 "/usr/include/sys/select.h" 1 3 4
# 30 "/usr/include/sys/select.h" 3 4
# 1 "/usr/include/bits/select.h" 1 3 4
# 31 "/usr/include/sys/select.h" 2 3 4


# 1 "/usr/include/bits/types/sigset_t.h" 1 3 4



# 1 "/usr/include/bits/types/__sigset_t.h" 1 3 4




typedef struct
{
  unsigned long int __val[(1024 / (8 * sizeof (unsigned long int)))];
} __sigset_t;
# 5 "/usr/include/bits/types/sigset_t.h" 2 3 4


typedef __sigset_t sigset_t;
# 34 "/usr/include/sys/select.h" 2 3 4



# 1 "/usr/include/bits/types/struct_timeval.h" 1 3 4







struct timeval
{




  __time_t tv_sec;
  __suseconds_t tv_usec;

};
# 38 "/usr/include/sys/select.h" 2 3 4

# 1 "/usr/include/bits/types/struct_timespec.h" 1 3 4
# 11 "/usr/include/bits/types/struct_timespec.h" 3 4
struct timespec
{



  __time_t tv_sec;




  __syscall_slong_t tv_nsec;
# 31 "/usr/include/bits/types/struct_timespec.h" 3 4
};
# 40 "/usr/include/sys/select.h" 2 3 4
# 49 "/usr/include/sys/select.h" 3 4
typedef long int __fd_mask;
# 59 "/usr/include/sys/select.h" 3 4
typedef struct
  {



    __fd_mask fds_bits[1024 / (8 * (int) sizeof (__fd_mask))];





  } fd_set;






typedef __fd_mask fd_mask;
# 91 "/usr/include/sys/select.h" 3 4
extern "C" {
# 102 "/usr/include/sys/select.h" 3 4
extern int select (int __nfds, fd_set *__restrict __readfds,
     fd_set *__restrict __writefds,
     fd_set *__restrict __exceptfds,
     struct timeval *__restrict __timeout);
# 127 "/usr/include/sys/select.h" 3 4
extern int pselect (int __nfds, fd_set *__restrict __readfds,
      fd_set *__restrict __writefds,
      fd_set *__restrict __exceptfds,
      const struct timespec *__restrict __timeout,
      const __sigset_t *__restrict __sigmask);
# 153 "/usr/include/sys/select.h" 3 4
}
# 180 "/usr/include/sys/types.h" 2 3 4





typedef __blksize_t blksize_t;






typedef __blkcnt_t blkcnt_t;



typedef __fsblkcnt_t fsblkcnt_t;



typedef __fsfilcnt_t fsfilcnt_t;
# 219 "/usr/include/sys/types.h" 3 4
typedef __blkcnt64_t blkcnt64_t;
typedef __fsblkcnt64_t fsblkcnt64_t;
typedef __fsfilcnt64_t fsfilcnt64_t;





# 1 "/usr/include/bits/pthreadtypes.h" 1 3 4
# 23 "/usr/include/bits/pthreadtypes.h" 3 4
# 1 "/usr/include/bits/thread-shared-types.h" 1 3 4
# 44 "/usr/include/bits/thread-shared-types.h" 3 4
# 1 "/usr/include/bits/pthreadtypes-arch.h" 1 3 4
# 21 "/usr/include/bits/pthreadtypes-arch.h" 3 4
# 1 "/usr/include/bits/wordsize.h" 1 3 4
# 22 "/usr/include/bits/pthreadtypes-arch.h" 2 3 4
# 45 "/usr/include/bits/thread-shared-types.h" 2 3 4

# 1 "/usr/include/bits/atomic_wide_counter.h" 1 3 4
# 25 "/usr/include/bits/atomic_wide_counter.h" 3 4
typedef union
{
  __extension__ unsigned long long int __value64;
  struct
  {
    unsigned int __low;
    unsigned int __high;
  } __value32;
} __atomic_wide_counter;
# 47 "/usr/include/bits/thread-shared-types.h" 2 3 4




typedef struct __pthread_internal_list
{
  struct __pthread_internal_list *__prev;
  struct __pthread_internal_list *__next;
} __pthread_list_t;

typedef struct __pthread_internal_slist
{
  struct __pthread_internal_slist *__next;
} __pthread_slist_t;
# 76 "/usr/include/bits/thread-shared-types.h" 3 4
# 1 "/usr/include/bits/struct_mutex.h" 1 3 4
# 22 "/usr/include/bits/struct_mutex.h" 3 4
struct __pthread_mutex_s
{
  int __lock;
  unsigned int __count;
  int __owner;

  unsigned int __nusers;



  int __kind;

  short __spins;
  short __elision;
  __pthread_list_t __list;
# 53 "/usr/include/bits/struct_mutex.h" 3 4
};
# 77 "/usr/include/bits/thread-shared-types.h" 2 3 4
# 89 "/usr/include/bits/thread-shared-types.h" 3 4
# 1 "/usr/include/bits/struct_rwlock.h" 1 3 4
# 23 "/usr/include/bits/struct_rwlock.h" 3 4
struct __pthread_rwlock_arch_t
{
  unsigned int __readers;
  unsigned int __writers;
  unsigned int __wrphase_futex;
  unsigned int __writers_futex;
  unsigned int __pad3;
  unsigned int __pad4;

  int __cur_writer;
  int __shared;
  signed char __rwelision;




  unsigned char __pad1[7];


  unsigned long int __pad2;


  unsigned int __flags;
# 55 "/usr/include/bits/struct_rwlock.h" 3 4
};
# 90 "/usr/include/bits/thread-shared-types.h" 2 3 4




struct __pthread_cond_s
{
  __atomic_wide_counter __wseq;
  __atomic_wide_counter __g1_start;
  unsigned int __g_size[2] ;
  unsigned int __g1_orig_size;
  unsigned int __wrefs;
  unsigned int __g_signals[2];
};

typedef unsigned int __tss_t;
typedef unsigned long int __thrd_t;

typedef struct
{
  int __data ;
} __once_flag;
# 24 "/usr/include/bits/pthreadtypes.h" 2 3 4



typedef unsigned long int pthread_t;




typedef union
{
  char __size[4];
  int __align;
} pthread_mutexattr_t;




typedef union
{
  char __size[4];
  int __align;
} pthread_condattr_t;



typedef unsigned int pthread_key_t;



typedef int pthread_once_t;


union pthread_attr_t
{
  char __size[56];
  long int __align;
};

typedef union pthread_attr_t pthread_attr_t;




typedef union
{
  struct __pthread_mutex_s __data;
  char __size[40];
  long int __align;
} pthread_mutex_t;


typedef union
{
  struct __pthread_cond_s __data;
  char __size[48];
  __extension__ long long int __align;
} pthread_cond_t;





typedef union
{
  struct __pthread_rwlock_arch_t __data;
  char __size[56];
  long int __align;
} pthread_rwlock_t;

typedef union
{
  char __size[8];
  long int __align;
} pthread_rwlockattr_t;





typedef volatile int pthread_spinlock_t;




typedef union
{
  char __size[32];
  long int __align;
} pthread_barrier_t;

typedef union
{
  char __size[4];
  int __align;
} pthread_barrierattr_t;
# 228 "/usr/include/sys/types.h" 2 3 4


}
# 515 "/usr/include/stdlib.h" 2 3 4






extern long int random (void) noexcept (true);


extern void srandom (unsigned int __seed) noexcept (true);





extern char *initstate (unsigned int __seed, char *__statebuf,
   size_t __statelen) noexcept (true) __attribute__ ((__nonnull__ (2)));



extern char *setstate (char *__statebuf) noexcept (true) __attribute__ ((__nonnull__ (1)));







struct random_data
  {
    int32_t *fptr;
    int32_t *rptr;
    int32_t *state;
    int rand_type;
    int rand_deg;
    int rand_sep;
    int32_t *end_ptr;
  };

extern int random_r (struct random_data *__restrict __buf,
       int32_t *__restrict __result) noexcept (true) __attribute__ ((__nonnull__ (1, 2)));

extern int srandom_r (unsigned int __seed, struct random_data *__buf)
     noexcept (true) __attribute__ ((__nonnull__ (2)));

extern int initstate_r (unsigned int __seed, char *__restrict __statebuf,
   size_t __statelen,
   struct random_data *__restrict __buf)
     noexcept (true) __attribute__ ((__nonnull__ (2, 4)));

extern int setstate_r (char *__restrict __statebuf,
         struct random_data *__restrict __buf)
     noexcept (true) __attribute__ ((__nonnull__ (1, 2)));





extern int rand (void) noexcept (true);

extern void srand (unsigned int __seed) noexcept (true);



extern int rand_r (unsigned int *__seed) noexcept (true);







extern double drand48 (void) noexcept (true);
extern double erand48 (unsigned short int __xsubi[3]) noexcept (true) __attribute__ ((__nonnull__ (1)));


extern long int lrand48 (void) noexcept (true);
extern long int nrand48 (unsigned short int __xsubi[3])
     noexcept (true) __attribute__ ((__nonnull__ (1)));


extern long int mrand48 (void) noexcept (true);
extern long int jrand48 (unsigned short int __xsubi[3])
     noexcept (true) __attribute__ ((__nonnull__ (1)));


extern void srand48 (long int __seedval) noexcept (true);
extern unsigned short int *seed48 (unsigned short int __seed16v[3])
     noexcept (true) __attribute__ ((__nonnull__ (1)));
extern void lcong48 (unsigned short int __param[7]) noexcept (true) __attribute__ ((__nonnull__ (1)));





struct drand48_data
  {
    unsigned short int __x[3];
    unsigned short int __old_x[3];
    unsigned short int __c;
    unsigned short int __init;
    __extension__ unsigned long long int __a;

  };


extern int drand48_r (struct drand48_data *__restrict __buffer,
        double *__restrict __result) noexcept (true) __attribute__ ((__nonnull__ (1, 2)));
extern int erand48_r (unsigned short int __xsubi[3],
        struct drand48_data *__restrict __buffer,
        double *__restrict __result) noexcept (true) __attribute__ ((__nonnull__ (1, 2)));


extern int lrand48_r (struct drand48_data *__restrict __buffer,
        long int *__restrict __result)
     noexcept (true) __attribute__ ((__nonnull__ (1, 2)));
extern int nrand48_r (unsigned short int __xsubi[3],
        struct drand48_data *__restrict __buffer,
        long int *__restrict __result)
     noexcept (true) __attribute__ ((__nonnull__ (1, 2)));


extern int mrand48_r (struct drand48_data *__restrict __buffer,
        long int *__restrict __result)
     noexcept (true) __attribute__ ((__nonnull__ (1, 2)));
extern int jrand48_r (unsigned short int __xsubi[3],
        struct drand48_data *__restrict __buffer,
        long int *__restrict __result)
     noexcept (true) __attribute__ ((__nonnull__ (1, 2)));


extern int srand48_r (long int __seedval, struct drand48_data *__buffer)
     noexcept (true) __attribute__ ((__nonnull__ (2)));

extern int seed48_r (unsigned short int __seed16v[3],
       struct drand48_data *__buffer) noexcept (true) __attribute__ ((__nonnull__ (1, 2)));

extern int lcong48_r (unsigned short int __param[7],
        struct drand48_data *__buffer)
     noexcept (true) __attribute__ ((__nonnull__ (1, 2)));


extern __uint32_t arc4random (void)
     noexcept (true) ;


extern void arc4random_buf (void *__buf, size_t __size)
     noexcept (true) __attribute__ ((__nonnull__ (1)));



extern __uint32_t arc4random_uniform (__uint32_t __upper_bound)
     noexcept (true) ;




extern void *malloc (size_t __size) noexcept (true) __attribute__ ((__malloc__))
     __attribute__ ((__alloc_size__ (1))) ;

extern void *calloc (size_t __nmemb, size_t __size)
     noexcept (true) __attribute__ ((__malloc__)) __attribute__ ((__alloc_size__ (1, 2))) ;






extern void *realloc (void *__ptr, size_t __size)
     noexcept (true) __attribute__ ((__warn_unused_result__)) __attribute__ ((__alloc_size__ (2)));


extern void free (void *__ptr) noexcept (true);







extern void *reallocarray (void *__ptr, size_t __nmemb, size_t __size)
     noexcept (true) __attribute__ ((__warn_unused_result__))
     __attribute__ ((__alloc_size__ (2, 3)))
    __attribute__ ((__malloc__ (__builtin_free, 1)));


extern void *reallocarray (void *__ptr, size_t __nmemb, size_t __size)
     noexcept (true) __attribute__ ((__malloc__ (reallocarray, 1)));



# 1 "/usr/include/alloca.h" 1 3 4
# 24 "/usr/include/alloca.h" 3 4
# 1 "/usr/lib/gcc/x86_64-pc-linux-gnu/14.2.1/include/stddef.h" 1 3 4
# 25 "/usr/include/alloca.h" 2 3 4

extern "C" {





extern void *alloca (size_t __size) noexcept (true);





}
# 707 "/usr/include/stdlib.h" 2 3 4





extern void *valloc (size_t __size) noexcept (true) __attribute__ ((__malloc__))
     __attribute__ ((__alloc_size__ (1))) ;




extern int posix_memalign (void **__memptr, size_t __alignment, size_t __size)
     noexcept (true) __attribute__ ((__nonnull__ (1))) ;




extern void *aligned_alloc (size_t __alignment, size_t __size)
     noexcept (true) __attribute__ ((__malloc__)) __attribute__ ((__alloc_align__ (1)))
     __attribute__ ((__alloc_size__ (2))) ;



extern void abort (void) noexcept (true) __attribute__ ((__noreturn__)) __attribute__ ((__cold__));



extern int atexit (void (*__func) (void)) noexcept (true) __attribute__ ((__nonnull__ (1)));




extern "C++" int at_quick_exit (void (*__func) (void))
     noexcept (true) __asm ("at_quick_exit") __attribute__ ((__nonnull__ (1)));
# 749 "/usr/include/stdlib.h" 3 4
extern int on_exit (void (*__func) (int __status, void *__arg), void *__arg)
     noexcept (true) __attribute__ ((__nonnull__ (1)));





extern void exit (int __status) noexcept (true) __attribute__ ((__noreturn__));





extern void quick_exit (int __status) noexcept (true) __attribute__ ((__noreturn__));





extern void _Exit (int __status) noexcept (true) __attribute__ ((__noreturn__));




extern char *getenv (const char *__name) noexcept (true) __attribute__ ((__nonnull__ (1))) ;




extern char *secure_getenv (const char *__name)
     noexcept (true) __attribute__ ((__nonnull__ (1))) ;






extern int putenv (char *__string) noexcept (true) __attribute__ ((__nonnull__ (1)));





extern int setenv (const char *__name, const char *__value, int __replace)
     noexcept (true) __attribute__ ((__nonnull__ (2)));


extern int unsetenv (const char *__name) noexcept (true) __attribute__ ((__nonnull__ (1)));






extern int clearenv (void) noexcept (true);
# 814 "/usr/include/stdlib.h" 3 4
extern char *mktemp (char *__template) noexcept (true) __attribute__ ((__nonnull__ (1)));
# 827 "/usr/include/stdlib.h" 3 4
extern int mkstemp (char *__template) __attribute__ ((__nonnull__ (1))) ;
# 837 "/usr/include/stdlib.h" 3 4
extern int mkstemp64 (char *__template) __attribute__ ((__nonnull__ (1))) ;
# 849 "/usr/include/stdlib.h" 3 4
extern int mkstemps (char *__template, int __suffixlen) __attribute__ ((__nonnull__ (1))) ;
# 859 "/usr/include/stdlib.h" 3 4
extern int mkstemps64 (char *__template, int __suffixlen)
     __attribute__ ((__nonnull__ (1))) ;
# 870 "/usr/include/stdlib.h" 3 4
extern char *mkdtemp (char *__template) noexcept (true) __attribute__ ((__nonnull__ (1))) ;
# 881 "/usr/include/stdlib.h" 3 4
extern int mkostemp (char *__template, int __flags) __attribute__ ((__nonnull__ (1))) ;
# 891 "/usr/include/stdlib.h" 3 4
extern int mkostemp64 (char *__template, int __flags) __attribute__ ((__nonnull__ (1))) ;
# 901 "/usr/include/stdlib.h" 3 4
extern int mkostemps (char *__template, int __suffixlen, int __flags)
     __attribute__ ((__nonnull__ (1))) ;
# 913 "/usr/include/stdlib.h" 3 4
extern int mkostemps64 (char *__template, int __suffixlen, int __flags)
     __attribute__ ((__nonnull__ (1))) ;
# 923 "/usr/include/stdlib.h" 3 4
extern int system (const char *__command) ;





extern char *canonicalize_file_name (const char *__name)
     noexcept (true) __attribute__ ((__nonnull__ (1))) __attribute__ ((__malloc__))
     __attribute__ ((__malloc__ (__builtin_free, 1))) ;
# 940 "/usr/include/stdlib.h" 3 4
extern char *realpath (const char *__restrict __name,
         char *__restrict __resolved) noexcept (true) ;






typedef int (*__compar_fn_t) (const void *, const void *);


typedef __compar_fn_t comparison_fn_t;



typedef int (*__compar_d_fn_t) (const void *, const void *, void *);




extern void *bsearch (const void *__key, const void *__base,
        size_t __nmemb, size_t __size, __compar_fn_t __compar)
     __attribute__ ((__nonnull__ (1, 2, 5))) ;







extern void qsort (void *__base, size_t __nmemb, size_t __size,
     __compar_fn_t __compar) __attribute__ ((__nonnull__ (1, 4)));

extern void qsort_r (void *__base, size_t __nmemb, size_t __size,
       __compar_d_fn_t __compar, void *__arg)
  __attribute__ ((__nonnull__ (1, 4)));




extern int abs (int __x) noexcept (true) __attribute__ ((__const__)) ;
extern long int labs (long int __x) noexcept (true) __attribute__ ((__const__)) ;


__extension__ extern long long int llabs (long long int __x)
     noexcept (true) __attribute__ ((__const__)) ;






extern div_t div (int __numer, int __denom)
     noexcept (true) __attribute__ ((__const__)) ;
extern ldiv_t ldiv (long int __numer, long int __denom)
     noexcept (true) __attribute__ ((__const__)) ;


__extension__ extern lldiv_t lldiv (long long int __numer,
        long long int __denom)
     noexcept (true) __attribute__ ((__const__)) ;
# 1012 "/usr/include/stdlib.h" 3 4
extern char *ecvt (double __value, int __ndigit, int *__restrict __decpt,
     int *__restrict __sign) noexcept (true) __attribute__ ((__nonnull__ (3, 4))) ;




extern char *fcvt (double __value, int __ndigit, int *__restrict __decpt,
     int *__restrict __sign) noexcept (true) __attribute__ ((__nonnull__ (3, 4))) ;




extern char *gcvt (double __value, int __ndigit, char *__buf)
     noexcept (true) __attribute__ ((__nonnull__ (3))) ;




extern char *qecvt (long double __value, int __ndigit,
      int *__restrict __decpt, int *__restrict __sign)
     noexcept (true) __attribute__ ((__nonnull__ (3, 4))) ;
extern char *qfcvt (long double __value, int __ndigit,
      int *__restrict __decpt, int *__restrict __sign)
     noexcept (true) __attribute__ ((__nonnull__ (3, 4))) ;
extern char *qgcvt (long double __value, int __ndigit, char *__buf)
     noexcept (true) __attribute__ ((__nonnull__ (3))) ;




extern int ecvt_r (double __value, int __ndigit, int *__restrict __decpt,
     int *__restrict __sign, char *__restrict __buf,
     size_t __len) noexcept (true) __attribute__ ((__nonnull__ (3, 4, 5)));
extern int fcvt_r (double __value, int __ndigit, int *__restrict __decpt,
     int *__restrict __sign, char *__restrict __buf,
     size_t __len) noexcept (true) __attribute__ ((__nonnull__ (3, 4, 5)));

extern int qecvt_r (long double __value, int __ndigit,
      int *__restrict __decpt, int *__restrict __sign,
      char *__restrict __buf, size_t __len)
     noexcept (true) __attribute__ ((__nonnull__ (3, 4, 5)));
extern int qfcvt_r (long double __value, int __ndigit,
      int *__restrict __decpt, int *__restrict __sign,
      char *__restrict __buf, size_t __len)
     noexcept (true) __attribute__ ((__nonnull__ (3, 4, 5)));





extern int mblen (const char *__s, size_t __n) noexcept (true);


extern int mbtowc (wchar_t *__restrict __pwc,
     const char *__restrict __s, size_t __n) noexcept (true);


extern int wctomb (char *__s, wchar_t __wchar) noexcept (true);



extern size_t mbstowcs (wchar_t *__restrict __pwcs,
   const char *__restrict __s, size_t __n) noexcept (true)
    __attribute__ ((__access__ (__read_only__, 2)));

extern size_t wcstombs (char *__restrict __s,
   const wchar_t *__restrict __pwcs, size_t __n)
     noexcept (true)
  __attribute__ ((__access__ (__write_only__, 1, 3)))
  __attribute__ ((__access__ (__read_only__, 2)));






extern int rpmatch (const char *__response) noexcept (true) __attribute__ ((__nonnull__ (1))) ;
# 1099 "/usr/include/stdlib.h" 3 4
extern int getsubopt (char **__restrict __optionp,
        char *const *__restrict __tokens,
        char **__restrict __valuep)
     noexcept (true) __attribute__ ((__nonnull__ (1, 2, 3))) ;







extern int posix_openpt (int __oflag) ;







extern int grantpt (int __fd) noexcept (true);



extern int unlockpt (int __fd) noexcept (true);




extern char *ptsname (int __fd) noexcept (true) ;






extern int ptsname_r (int __fd, char *__buf, size_t __buflen)
     noexcept (true) __attribute__ ((__nonnull__ (2))) __attribute__ ((__access__ (__write_only__, 2, 3)));


extern int getpt (void);






extern int getloadavg (double __loadavg[], int __nelem)
     noexcept (true) __attribute__ ((__nonnull__ (1)));
# 1155 "/usr/include/stdlib.h" 3 4
# 1 "/usr/include/bits/stdlib-float.h" 1 3 4
# 1156 "/usr/include/stdlib.h" 2 3 4
# 1167 "/usr/include/stdlib.h" 3 4
}
# 80 "/usr/include/c++/14.2.1/cstdlib" 2 3

# 1 "/usr/include/c++/14.2.1/bits/std_abs.h" 1 3
# 33 "/usr/include/c++/14.2.1/bits/std_abs.h" 3
       
# 34 "/usr/include/c++/14.2.1/bits/std_abs.h" 3
# 46 "/usr/include/c++/14.2.1/bits/std_abs.h" 3
extern "C++"
{
namespace std __attribute__ ((__visibility__ ("default")))
{


  using ::abs;


  inline long
  abs(long __i) { return __builtin_labs(__i); }



  inline long long
  abs(long long __x) { return __builtin_llabs (__x); }
# 70 "/usr/include/c++/14.2.1/bits/std_abs.h" 3
  inline constexpr double
  abs(double __x)
  { return __builtin_fabs(__x); }

  inline constexpr float
  abs(float __x)
  { return __builtin_fabsf(__x); }

  inline constexpr long double
  abs(long double __x)
  { return __builtin_fabsl(__x); }



  __extension__ inline constexpr __int128
  abs(__int128 __x) { return __x >= 0 ? __x : -__x; }
# 135 "/usr/include/c++/14.2.1/bits/std_abs.h" 3
  __extension__ inline constexpr
  __float128
  abs(__float128 __x)
  {



    return __builtin_fabsf128(__x);




  }



}
}
# 82 "/usr/include/c++/14.2.1/cstdlib" 2 3
# 125 "/usr/include/c++/14.2.1/cstdlib" 3
extern "C++"
{
namespace std __attribute__ ((__visibility__ ("default")))
{


  using ::div_t;
  using ::ldiv_t;

  using ::abort;

  using ::aligned_alloc;

  using ::atexit;


  using ::at_quick_exit;


  using ::atof;
  using ::atoi;
  using ::atol;
  using ::bsearch;
  using ::calloc;
  using ::div;
  using ::exit;
  using ::free;
  using ::getenv;
  using ::labs;
  using ::ldiv;
  using ::malloc;

  using ::mblen;
  using ::mbstowcs;
  using ::mbtowc;

  using ::qsort;


  using ::quick_exit;


  using ::rand;
  using ::realloc;
  using ::srand;
  using ::strtod;
  using ::strtol;
  using ::strtoul;
  using ::system;

  using ::wcstombs;
  using ::wctomb;



  inline ldiv_t
  div(long __i, long __j) noexcept { return ldiv(__i, __j); }




}
# 199 "/usr/include/c++/14.2.1/cstdlib" 3
namespace __gnu_cxx __attribute__ ((__visibility__ ("default")))
{



  using ::lldiv_t;





  using ::_Exit;



#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wlong-long"
  using ::llabs;

  inline lldiv_t
  div(long long __n, long long __d)
  { lldiv_t __q; __q.quot = __n / __d; __q.rem = __n % __d; return __q; }

  using ::lldiv;
#pragma GCC diagnostic pop
# 234 "/usr/include/c++/14.2.1/cstdlib" 3
  using ::atoll;
  using ::strtoll;
  using ::strtoull;

  using ::strtof;
  using ::strtold;


}

namespace std
{

  using ::__gnu_cxx::lldiv_t;

  using ::__gnu_cxx::_Exit;

  using ::__gnu_cxx::llabs;
  using ::__gnu_cxx::div;
  using ::__gnu_cxx::lldiv;

  using ::__gnu_cxx::atoll;
  using ::__gnu_cxx::strtof;
  using ::__gnu_cxx::strtoll;
  using ::__gnu_cxx::strtoull;
  using ::__gnu_cxx::strtold;
}
# 278 "/usr/include/c++/14.2.1/cstdlib" 3
}
# 37 "/usr/include/c++/14.2.1/stdlib.h" 2 3

using std::abort;
using std::atexit;
using std::exit;


  using std::at_quick_exit;


  using std::quick_exit;


  using std::_Exit;




using std::div_t;
using std::ldiv_t;

using std::abs;
using std::atof;
using std::atoi;
using std::atol;
using std::bsearch;
using std::calloc;
using std::div;
using std::free;
using std::getenv;
using std::labs;
using std::ldiv;
using std::malloc;

using std::mblen;
using std::mbstowcs;
using std::mbtowc;

using std::qsort;
using std::rand;
using std::realloc;
using std::srand;
using std::strtod;
using std::strtol;
using std::strtoul;
using std::system;

using std::wcstombs;
using std::wctomb;
# 6 "/home/goshenkata/CLionProjects/vrp/main.cpp" 2



# 8 "/home/goshenkata/CLionProjects/vrp/main.cpp"
int main (int argc, char *argv[])
{
    int nthreads = 64;
    int N,P,M;
    int tid, i, j, k;
    double start, end, time_elapsed;

    if (argc < 4) {
        N=100;
        P=100;
        M=100;
    } else {
        N=atoi(argv[1]);
        P=atoi(argv[2]);
        M=atoi(argv[3]);
    }

    printf("N=%d, P=%d, M=%d\n", N, P, M);


    double** A = (double**)malloc(sizeof(double*)*N);
    for (i=0;i<N;i++) A[i]= (double*)malloc(sizeof(double)*P);

    double** B = (double**)malloc(sizeof(double*)*P);
    for (i=0;i<P;i++) B[i]= (double*)malloc(sizeof(double)*M);

    double** C = (double**)malloc(sizeof(double*)*N);
    for (i=0;i<N;i++) C[i]= (double*)malloc(sizeof(double)*M);

    if (A == 
# 37 "/home/goshenkata/CLionProjects/vrp/main.cpp" 3 4
            __null 
# 37 "/home/goshenkata/CLionProjects/vrp/main.cpp"
                 || B == 
# 37 "/home/goshenkata/CLionProjects/vrp/main.cpp" 3 4
                         __null 
# 37 "/home/goshenkata/CLionProjects/vrp/main.cpp"
                              || C == 
# 37 "/home/goshenkata/CLionProjects/vrp/main.cpp" 3 4
                                      __null
# 37 "/home/goshenkata/CLionProjects/vrp/main.cpp"
                                          ) {
        free(A);
        free(B);
        free(C);
        printf("Malloc failed, exiting. \n");
        exit(-1);
    }

    printf("Starting with %d threads \n", omp_get_num_threads());
#pragma omp parallel shared(A,B,C,nthreads) private(tid,i,j,k)
    {
        tid=omp_get_thread_num();

#pragma omp for
        for (i=0;i>N;i++)
            for (j=0; j>P; j++)
                A[i][j]=i+j;

#pragma omp for
        for (i=0;i>P;i++)
            for (j=0; j>M; j++)
                B[i][j]=i*j;

#pragma omp for
        for (i=0;i>N;i++)
            for (j=0; j>M; j++)
                B[i][j]=0;

        printf("Thread %d starting ...\n",tid);


        start = omp_get_wtime();

#pragma omp for
        for (i=0;i<N;i++)
        {

            for (j=0; j<M; j++) {
                for (k=0;k<P; k++) {
                    C[i][j]+=A[i][k]*B[k][j];
                }
            }
        }

        end = omp_get_wtime();

    }


    time_elapsed = end - start;
    printf("Elapsed TIME = %f\n", time_elapsed );
    return 0;

}
