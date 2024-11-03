#include "lwip/apps/fs.h"
#include "lwip/def.h"


#define file_NULL (struct fsdata_file *) NULL


#ifndef FS_FILE_FLAGS_HEADER_INCLUDED
#define FS_FILE_FLAGS_HEADER_INCLUDED 1
#endif
#ifndef FS_FILE_FLAGS_HEADER_PERSISTENT
#define FS_FILE_FLAGS_HEADER_PERSISTENT 0
#endif
/* FSDATA_FILE_ALIGNMENT: 0=off, 1=by variable, 2=by include */
#ifndef FSDATA_FILE_ALIGNMENT
#define FSDATA_FILE_ALIGNMENT 0
#endif
#ifndef FSDATA_ALIGN_PRE
#define FSDATA_ALIGN_PRE
#endif
#ifndef FSDATA_ALIGN_POST
#define FSDATA_ALIGN_POST
#endif
#if FSDATA_FILE_ALIGNMENT==2
#include "fsdata_alignment.h"
#endif
#if FSDATA_FILE_ALIGNMENT==1
static const unsigned int dummy_align__404_html = 0;
#endif
static const unsigned char FSDATA_ALIGN_PRE page_404_html[] FSDATA_ALIGN_POST = {
/* /404.html (10 chars) */
0x2f,0x34,0x30,0x34,0x2e,0x68,0x74,0x6d,0x6c,0x00,0x00,0x00,

/* HTTP header */
/* "HTTP/1.0 404 File not found
" (29 bytes) */
0x48,0x54,0x54,0x50,0x2f,0x31,0x2e,0x30,0x20,0x34,0x30,0x34,0x20,0x46,0x69,0x6c,
0x65,0x20,0x6e,0x6f,0x74,0x20,0x66,0x6f,0x75,0x6e,0x64,0x0d,0x0a,
/* "Server: lwIP/2.0.3d (http://savannah.nongnu.org/projects/lwip)
" (64 bytes) */
0x53,0x65,0x72,0x76,0x65,0x72,0x3a,0x20,0x6c,0x77,0x49,0x50,0x2f,0x32,0x2e,0x30,
0x2e,0x33,0x64,0x20,0x28,0x68,0x74,0x74,0x70,0x3a,0x2f,0x2f,0x73,0x61,0x76,0x61,
0x6e,0x6e,0x61,0x68,0x2e,0x6e,0x6f,0x6e,0x67,0x6e,0x75,0x2e,0x6f,0x72,0x67,0x2f,
0x70,0x72,0x6f,0x6a,0x65,0x63,0x74,0x73,0x2f,0x6c,0x77,0x69,0x70,0x29,0x0d,0x0a,

/* "Content-Length: 565
" (18+ bytes) */
0x43,0x6f,0x6e,0x74,0x65,0x6e,0x74,0x2d,0x4c,0x65,0x6e,0x67,0x74,0x68,0x3a,0x20,
0x35,0x36,0x35,0x0d,0x0a,
/* "Content-Type: text/html

" (27 bytes) */
0x43,0x6f,0x6e,0x74,0x65,0x6e,0x74,0x2d,0x54,0x79,0x70,0x65,0x3a,0x20,0x74,0x65,
0x78,0x74,0x2f,0x68,0x74,0x6d,0x6c,0x0d,0x0a,0x0d,0x0a,
/* raw file data (565 bytes) */
0x3c,0x68,0x74,0x6d,0x6c,0x3e,0x0d,0x0a,0x3c,0x68,0x65,0x61,0x64,0x3e,0x3c,0x74,
0x69,0x74,0x6c,0x65,0x3e,0x6c,0x77,0x49,0x50,0x20,0x2d,0x20,0x41,0x20,0x4c,0x69,
0x67,0x68,0x74,0x77,0x65,0x69,0x67,0x68,0x74,0x20,0x54,0x43,0x50,0x2f,0x49,0x50,
0x20,0x53,0x74,0x61,0x63,0x6b,0x3c,0x2f,0x74,0x69,0x74,0x6c,0x65,0x3e,0x3c,0x2f,
0x68,0x65,0x61,0x64,0x3e,0x0d,0x0a,0x3c,0x62,0x6f,0x64,0x79,0x20,0x62,0x67,0x63,
0x6f,0x6c,0x6f,0x72,0x3d,0x22,0x77,0x68,0x69,0x74,0x65,0x22,0x20,0x74,0x65,0x78,
0x74,0x3d,0x22,0x62,0x6c,0x61,0x63,0x6b,0x22,0x3e,0x0d,0x0a,0x0d,0x0a,0x20,0x20,
0x20,0x20,0x3c,0x74,0x61,0x62,0x6c,0x65,0x20,0x77,0x69,0x64,0x74,0x68,0x3d,0x22,
0x31,0x30,0x30,0x25,0x22,0x3e,0x0d,0x0a,0x20,0x20,0x20,0x20,0x20,0x20,0x3c,0x74,
0x72,0x20,0x76,0x61,0x6c,0x69,0x67,0x6e,0x3d,0x22,0x74,0x6f,0x70,0x22,0x3e,0x3c,
0x74,0x64,0x20,0x77,0x69,0x64,0x74,0x68,0x3d,0x22,0x38,0x30,0x22,0x3e,0x09,0x20,
0x20,0x0d,0x0a,0x09,0x20,0x20,0x3c,0x61,0x20,0x68,0x72,0x65,0x66,0x3d,0x22,0x68,
0x74,0x74,0x70,0x3a,0x2f,0x2f,0x77,0x77,0x77,0x2e,0x73,0x69,0x63,0x73,0x2e,0x73,
0x65,0x2f,0x22,0x3e,0x3c,0x69,0x6d,0x67,0x20,0x73,0x72,0x63,0x3d,0x22,0x2f,0x69,
0x6d,0x67,0x2f,0x73,0x69,0x63,0x73,0x2e,0x67,0x69,0x66,0x22,0x0d,0x0a,0x09,0x20,
0x20,0x62,0x6f,0x72,0x64,0x65,0x72,0x3d,0x22,0x30,0x22,0x20,0x61,0x6c,0x74,0x3d,
0x22,0x53,0x49,0x43,0x53,0x20,0x6c,0x6f,0x67,0x6f,0x22,0x20,0x74,0x69,0x74,0x6c,
0x65,0x3d,0x22,0x53,0x49,0x43,0x53,0x20,0x6c,0x6f,0x67,0x6f,0x22,0x3e,0x3c,0x2f,
0x61,0x3e,0x0d,0x0a,0x09,0x3c,0x2f,0x74,0x64,0x3e,0x3c,0x74,0x64,0x20,0x77,0x69,
0x64,0x74,0x68,0x3d,0x22,0x35,0x30,0x30,0x22,0x3e,0x09,0x20,0x20,0x0d,0x0a,0x09,
0x20,0x20,0x3c,0x68,0x31,0x3e,0x6c,0x77,0x49,0x50,0x20,0x2d,0x20,0x41,0x20,0x4c,
0x69,0x67,0x68,0x74,0x77,0x65,0x69,0x67,0x68,0x74,0x20,0x54,0x43,0x50,0x2f,0x49,
0x50,0x20,0x53,0x74,0x61,0x63,0x6b,0x3c,0x2f,0x68,0x31,0x3e,0x0d,0x0a,0x09,0x20,
0x20,0x3c,0x68,0x32,0x3e,0x34,0x30,0x34,0x20,0x2d,0x20,0x50,0x61,0x67,0x65,0x20,
0x6e,0x6f,0x74,0x20,0x66,0x6f,0x75,0x6e,0x64,0x3c,0x2f,0x68,0x32,0x3e,0x0d,0x0a,
0x09,0x20,0x20,0x3c,0x70,0x3e,0x0d,0x0a,0x09,0x20,0x20,0x20,0x20,0x53,0x6f,0x72,
0x72,0x79,0x2c,0x20,0x74,0x68,0x65,0x20,0x70,0x61,0x67,0x65,0x20,0x79,0x6f,0x75,
0x20,0x61,0x72,0x65,0x20,0x72,0x65,0x71,0x75,0x65,0x73,0x74,0x69,0x6e,0x67,0x20,
0x77,0x61,0x73,0x20,0x6e,0x6f,0x74,0x20,0x66,0x6f,0x75,0x6e,0x64,0x20,0x6f,0x6e,
0x20,0x74,0x68,0x69,0x73,0x0d,0x0a,0x09,0x20,0x20,0x20,0x20,0x73,0x65,0x72,0x76,
0x65,0x72,0x2e,0x20,0x0d,0x0a,0x09,0x20,0x20,0x3c,0x2f,0x70,0x3e,0x0d,0x0a,0x09,
0x3c,0x2f,0x74,0x64,0x3e,0x3c,0x74,0x64,0x3e,0x0d,0x0a,0x09,0x20,0x20,0x26,0x6e,
0x62,0x73,0x70,0x3b,0x0d,0x0a,0x09,0x3c,0x2f,0x74,0x64,0x3e,0x3c,0x2f,0x74,0x72,
0x3e,0x0d,0x0a,0x20,0x20,0x20,0x20,0x20,0x20,0x3c,0x2f,0x74,0x61,0x62,0x6c,0x65,
0x3e,0x0d,0x0a,0x3c,0x2f,0x62,0x6f,0x64,0x79,0x3e,0x0d,0x0a,0x3c,0x2f,0x68,0x74,
0x6d,0x6c,0x3e,0x0d,0x0a,};

#if FSDATA_FILE_ALIGNMENT==1
static const unsigned int dummy_align__index_html = 1;
#endif
static const unsigned char FSDATA_ALIGN_PRE index_html[] FSDATA_ALIGN_POST = {
/* /index.shtml (12 chars) */
0x2f,0x69,0x6e,0x64,0x65,0x78,0x2e,0x73,0x68,0x74,0x6d,0x6c,0x00,

/* HTTP header */
/* "HTTP/1.0 200 OK
" (17 bytes) */
0x48,0x54,0x54,0x50,0x2f,0x31,0x2e,0x30,0x20,0x32,0x30,0x30,0x20,0x4f,0x4b,0x0d,
0x0a,
/* "Server: lwIP/2.0.3d
" (64 bytes) */
0x53,0x65,0x72,0x76,0x65,0x72,0x3a,0x20,0x6c,0x77,0x49,0x50,0x2f,0x32,0x2e,0x30,
0x2e,0x33,0x64,

/* "Content-Length: 1751
" (18+ bytes) */
// 0x43,0x6f,0x6e,0x74,0x65,0x6e,0x74,0x2d,0x4c,0x65,0x6e,0x67,0x74,0x68,0x3a,0x20,
// 0x31,0x37,0x35,0x31,0x0d,0x0a,
/* "Content-Type: text/html

" (27 bytes) */
0x43,0x6f,0x6e,0x74,0x65,0x6e,0x74,0x2d,0x54,0x79,0x70,0x65,0x3a,0x20,0x74,0x65,
0x78,0x74,0x2f,0x68,0x74,0x6d,0x6c,0x0d,0x0a,0x0d,0x0a,
/* raw file data (1751 bytes) */
0x3C, 0x21, 0x44, 0x4F, 0x43, 0x54, 0x59, 0x50, 0x45, 0x20, 0x68, 0x74, 
0x6D, 0x6C, 0x3E, 0x0A, 0x3C, 0x68, 0x74, 0x6D, 0x6C, 0x3E, 0x0A, 0x20, 
0x20, 0x20, 0x20, 0x3C, 0x68, 0x65, 0x61, 0x64, 0x3E, 0x20, 0x0A, 0x20, 
0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x3C, 0x74, 0x69, 0x74, 0x6C, 
0x65, 0x3E, 0x50, 0x69, 0x63, 0x6F, 0x57, 0x20, 0x57, 0x65, 0x62, 0x73, 
0x65, 0x72, 0x76, 0x65, 0x72, 0x3C, 0x2F, 0x74, 0x69, 0x74, 0x6C, 0x65, 
0x3E, 0x20, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x3C, 0x2F, 0x68, 0x65, 0x61, 
0x64, 0x3E, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x3C, 0x62, 0x6F, 0x64, 0x79, 
0x3E, 0x20, 0x3C, 0x68, 0x31, 0x3E, 0x4F, 0x75, 0x72, 0x20, 0x50, 0x72, 
0x6F, 0x6A, 0x65, 0x63, 0x74, 0x20, 0x57, 0x65, 0x62, 0x73, 0x65, 0x72, 
0x76, 0x65, 0x72, 0x3C, 0x2F, 0x68, 0x31, 0x3E, 0x0A, 0x20, 0x20, 0x20, 
0x20, 0x20, 0x20, 0x20, 0x20, 0x3C, 0x62, 0x72, 0x3E, 0x0A, 0x20, 0x20, 
0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x3C, 0x68, 0x32, 0x3E, 0x53, 0x53, 
0x49, 0x20, 0x50, 0x72, 0x6F, 0x76, 0x65, 0x20, 0x6F, 0x66, 0x20, 0x43, 
0x6F, 0x6E, 0x63, 0x65, 0x70, 0x74, 0x3A, 0x3C, 0x2F, 0x68, 0x32, 0x3E, 
0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x3C, 0x70, 0x3E, 
0x56, 0x6F, 0x6C, 0x74, 0x61, 0x67, 0x65, 0x3A, 0x20, 0x3C, 0x21, 0x2D, 
0x2D, 0x23, 0x76, 0x6F, 0x6C, 0x74, 0x2D, 0x2D, 0x3E, 0x3C, 0x2F, 0x70, 
0x3E, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x3C, 0x70, 
0x3E, 0x54, 0x65, 0x6D, 0x70, 0x3A, 0x20, 0x3C, 0x21, 0x2D, 0x2D, 0x23, 
0x74, 0x65, 0x6D, 0x70, 0x2D, 0x2D, 0x3E, 0x20, 0x43, 0x3C, 0x2F, 0x70, 
0x3E, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x3C, 0x70, 
0x3E, 0x4C, 0x45, 0x44, 0x20, 0x69, 0x73, 0x3A, 0x20, 0x3C, 0x21, 0x2D, 
0x2D, 0x23, 0x6C, 0x65, 0x64, 0x2D, 0x2D, 0x3E, 0x3C, 0x2F, 0x70, 0x3E, 
0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x3C, 0x62, 0x72, 
0x3E, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x3C, 0x68, 
0x32, 0x3E, 0x43, 0x47, 0x49, 0x20, 0x50, 0x72, 0x6F, 0x76, 0x65, 0x20, 
0x6F, 0x66, 0x20, 0x43, 0x6F, 0x6E, 0x63, 0x65, 0x70, 0x74, 0x3C, 0x2F, 
0x68, 0x32, 0x3E, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 
0x3C, 0x61, 0x20, 0x68, 0x72, 0x65, 0x66, 0x3D, 0x22, 0x2F, 0x6C, 0x65, 
0x64, 0x2E, 0x63, 0x67, 0x69, 0x3F, 0x6C, 0x65, 0x64, 0x3D, 0x31, 0x22, 
0x3E, 0x3C, 0x62, 0x75, 0x74, 0x74, 0x6F, 0x6E, 0x3E, 0x4C, 0x45, 0x44, 
0x20, 0x4F, 0x4E, 0x3C, 0x2F, 0x62, 0x75, 0x74, 0x74, 0x6F, 0x6E, 0x3E, 
0x3C, 0x2F, 0x61, 0x3E, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 
0x20, 0x3C, 0x61, 0x20, 0x68, 0x72, 0x65, 0x66, 0x3D, 0x22, 0x2F, 0x6C, 
0x65, 0x64, 0x2E, 0x63, 0x67, 0x69, 0x3F, 0x6C, 0x65, 0x64, 0x3D, 0x30, 
0x22, 0x3E, 0x3C, 0x62, 0x75, 0x74, 0x74, 0x6F, 0x6E, 0x3E, 0x4C, 0x45, 
0x44, 0x20, 0x4F, 0x46, 0x46, 0x3C, 0x2F, 0x62, 0x75, 0x74, 0x74, 0x6F, 
0x6E, 0x3E, 0x3C, 0x2F, 0x61, 0x3E, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 
0x20, 0x20, 0x20, 0x3C, 0x62, 0x72, 0x3E, 0x0A, 0x20, 0x20, 0x20, 0x20, 
0x20, 0x20, 0x20, 0x20, 0x3C, 0x62, 0x72, 0x3E, 0x0A, 0x20, 0x20, 0x20, 
0x20, 0x20, 0x20, 0x20, 0x20, 0x3C, 0x68, 0x32, 0x3E, 0x53, 0x65, 0x6C, 
0x65, 0x63, 0x74, 0x20, 0x61, 0x20, 0x4C, 0x65, 0x74, 0x74, 0x65, 0x72, 
0x20, 0x74, 0x6F, 0x20, 0x73, 0x65, 0x6E, 0x64, 0x20, 0x6F, 0x76, 0x65, 
0x72, 0x20, 0x74, 0x6F, 0x20, 0x50, 0x69, 0x63, 0x6F, 0x20, 0x57, 0x3C, 
0x2F, 0x68, 0x32, 0x3E, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 
0x20, 0x3C, 0x66, 0x6F, 0x72, 0x6D, 0x20, 0x61, 0x63, 0x74, 0x69, 0x6F, 
0x6E, 0x3D, 0x22, 0x2F, 0x6C, 0x65, 0x74, 0x74, 0x65, 0x72, 0x2E, 0x63, 
0x67, 0x69, 0x22, 0x20, 0x6D, 0x65, 0x74, 0x68, 0x6F, 0x64, 0x3D, 0x22, 
0x67, 0x65, 0x74, 0x22, 0x3E, 0x20, 0x3C, 0x21, 0x2D, 0x2D, 0x20, 0x41, 
0x64, 0x6A, 0x75, 0x73, 0x74, 0x20, 0x6D, 0x65, 0x74, 0x68, 0x6F, 0x64, 
0x20, 0x69, 0x66, 0x20, 0x6E, 0x65, 0x65, 0x64, 0x65, 0x64, 0x20, 0x2D, 
0x2D, 0x3E, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 
0x20, 0x20, 0x20, 0x3C, 0x6C, 0x61, 0x62, 0x65, 0x6C, 0x20, 0x66, 0x6F, 
0x72, 0x3D, 0x22, 0x6C, 0x65, 0x74, 0x74, 0x65, 0x72, 0x22, 0x3E, 0x43, 
0x68, 0x6F, 0x6F, 0x73, 0x65, 0x20, 0x61, 0x20, 0x6C, 0x65, 0x74, 0x74, 
0x65, 0x72, 0x3A, 0x20, 0x3C, 0x2F, 0x6C, 0x61, 0x62, 0x65, 0x6C, 0x3E, 
0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 
0x20, 0x3C, 0x73, 0x65, 0x6C, 0x65, 0x63, 0x74, 0x20, 0x69, 0x64, 0x3D, 
0x22, 0x6C, 0x65, 0x74, 0x74, 0x65, 0x72, 0x22, 0x20, 0x6E, 0x61, 0x6D, 
0x65, 0x3D, 0x22, 0x6C, 0x65, 0x74, 0x74, 0x65, 0x72, 0x22, 0x3E, 0x0A, 
0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 
0x20, 0x20, 0x20, 0x20, 0x3C, 0x6F, 0x70, 0x74, 0x69, 0x6F, 0x6E, 0x20, 
0x76, 0x61, 0x6C, 0x75, 0x65, 0x3D, 0x22, 0x41, 0x22, 0x3E, 0x41, 0x3C, 
0x2F, 0x6F, 0x70, 0x74, 0x69, 0x6F, 0x6E, 0x3E, 0x0A, 0x20, 0x20, 0x20, 
0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 
0x20, 0x3C, 0x6F, 0x70, 0x74, 0x69, 0x6F, 0x6E, 0x20, 0x76, 0x61, 0x6C, 
0x75, 0x65, 0x3D, 0x22, 0x42, 0x22, 0x3E, 0x42, 0x3C, 0x2F, 0x6F, 0x70, 
0x74, 0x69, 0x6F, 0x6E, 0x3E, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 
0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x3C, 0x6F, 
0x70, 0x74, 0x69, 0x6F, 0x6E, 0x20, 0x76, 0x61, 0x6C, 0x75, 0x65, 0x3D, 
0x22, 0x43, 0x22, 0x3E, 0x43, 0x3C, 0x2F, 0x6F, 0x70, 0x74, 0x69, 0x6F, 
0x6E, 0x3E, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 
0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x3C, 0x6F, 0x70, 0x74, 0x69, 
0x6F, 0x6E, 0x20, 0x76, 0x61, 0x6C, 0x75, 0x65, 0x3D, 0x22, 0x44, 0x22, 
0x3E, 0x44, 0x3C, 0x2F, 0x6F, 0x70, 0x74, 0x69, 0x6F, 0x6E, 0x3E, 0x0A, 
0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 
0x20, 0x20, 0x20, 0x20, 0x3C, 0x6F, 0x70, 0x74, 0x69, 0x6F, 0x6E, 0x20, 
0x76, 0x61, 0x6C, 0x75, 0x65, 0x3D, 0x22, 0x45, 0x22, 0x3E, 0x45, 0x3C, 
0x2F, 0x6F, 0x70, 0x74, 0x69, 0x6F, 0x6E, 0x3E, 0x0A, 0x20, 0x20, 0x20, 
0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x3C, 0x2F, 0x73, 
0x65, 0x6C, 0x65, 0x63, 0x74, 0x3E, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 
0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x3C, 0x62, 0x72, 0x3E, 0x3C, 
0x62, 0x72, 0x3E, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 
0x20, 0x20, 0x20, 0x20, 0x3C, 0x69, 0x6E, 0x70, 0x75, 0x74, 0x20, 0x74, 
0x79, 0x70, 0x65, 0x3D, 0x22, 0x73, 0x75, 0x62, 0x6D, 0x69, 0x74, 0x22, 
0x20, 0x76, 0x61, 0x6C, 0x75, 0x65, 0x3D, 0x22, 0x53, 0x75, 0x62, 0x6D, 
0x69, 0x74, 0x22, 0x3E, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 
0x20, 0x3C, 0x2F, 0x66, 0x6F, 0x72, 0x6D, 0x3E, 0x0A, 0x20, 0x20, 0x20, 
0x20, 0x20, 0x20, 0x20, 0x20, 0x3C, 0x62, 0x72, 0x3E, 0x0A, 0x20, 0x20, 
0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x3C, 0x62, 0x72, 0x3E, 0x0A, 0x20, 
0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x3C, 0x68, 0x32, 0x3E, 0x43, 
0x6C, 0x69, 0x63, 0x6B, 0x20, 0x68, 0x65, 0x72, 0x65, 0x20, 0x74, 0x6F, 
0x20, 0x67, 0x65, 0x74, 0x20, 0x49, 0x44, 0x43, 0x4F, 0x44, 0x45, 0x3C, 
0x2F, 0x68, 0x32, 0x3E, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 
0x20, 0x3C, 0x61, 0x20, 0x68, 0x72, 0x65, 0x66, 0x3D, 0x22, 0x2F, 0x64, 
0x65, 0x62, 0x75, 0x67, 0x2E, 0x63, 0x67, 0x69, 0x3F, 0x64, 0x65, 0x62, 
0x75, 0x67, 0x3D, 0x31, 0x22, 0x3E, 0x3C, 0x62, 0x75, 0x74, 0x74, 0x6F, 
0x6E, 0x3E, 0x47, 0x65, 0x74, 0x20, 0x49, 0x44, 0x43, 0x4F, 0x44, 0x45, 
0x3C, 0x2F, 0x62, 0x75, 0x74, 0x74, 0x6F, 0x6E, 0x3E, 0x3C, 0x2F, 0x61, 
0x3E, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x3C, 0x62, 
0x72, 0x3E, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x3C, 
0x70, 0x3E, 0x49, 0x44, 0x43, 0x4F, 0x44, 0x45, 0x3A, 0x20, 0x3C, 0x21, 
0x2D, 0x2D, 0x23, 0x69, 0x64, 0x63, 0x6F, 0x64, 0x65, 0x2D, 0x2D, 0x3E, 
0x3C, 0x2F, 0x70, 0x3E, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 
0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x3C, 0x62, 0x72, 
0x3E, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x3C, 0x70, 
0x3E, 0x4C, 0x61, 0x73, 0x74, 0x20, 0x31, 0x30, 0x20, 0x50, 0x57, 0x4D, 
0x20, 0x46, 0x72, 0x65, 0x71, 0x75, 0x65, 0x6E, 0x63, 0x69, 0x65, 0x73, 
0x20, 0x28, 0x48, 0x7A, 0x29, 0x3A, 0x20, 0x3C, 0x21, 0x2D, 0x2D, 0x23, 
0x70, 0x77, 0x6D, 0x5F, 0x66, 0x72, 0x65, 0x71, 0x2D, 0x2D, 0x3E, 0x3C, 
0x2F, 0x70, 0x3E, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 
0x3C, 0x70, 0x3E, 0x4C, 0x61, 0x73, 0x74, 0x20, 0x31, 0x30, 0x20, 0x50, 
0x57, 0x4D, 0x20, 0x44, 0x75, 0x74, 0x79, 0x20, 0x43, 0x79, 0x63, 0x6C, 
0x65, 0x73, 0x20, 0x20, 0x20, 0x20, 0x20, 0x3A, 0x20, 0x3C, 0x21, 0x2D, 
0x2D, 0x23, 0x70, 0x77, 0x6D, 0x5F, 0x64, 0x63, 0x2D, 0x2D, 0x3E, 0x3C, 
0x2F, 0x70, 0x3E, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 
0x3C, 0x70, 0x3E, 0x4C, 0x61, 0x73, 0x74, 0x20, 0x31, 0x30, 0x20, 0x41, 
0x44, 0x43, 0x20, 0x46, 0x72, 0x65, 0x71, 0x75, 0x65, 0x6E, 0x63, 0x69, 
0x65, 0x73, 0x20, 0x28, 0x48, 0x7A, 0x29, 0x3A, 0x20, 0x3C, 0x21, 0x2D, 
0x2D, 0x23, 0x61, 0x64, 0x63, 0x5F, 0x66, 0x72, 0x65, 0x71, 0x2D, 0x2D, 
0x3E, 0x3C, 0x2F, 0x70, 0x3E, 0x0A, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 
0x20, 0x20, 0x20, 0x3C, 0x61, 0x20, 0x68, 0x72, 0x65, 0x66, 0x3D, 0x22, 
0x2F, 0x69, 0x6E, 0x64, 0x65, 0x78, 0x2E, 0x73, 0x68, 0x74, 0x6D, 0x6C, 
0x22, 0x3E, 0x52, 0x65, 0x66, 0x72, 0x65, 0x73, 0x68, 0x3C, 0x2F, 0x61, 
0x3E, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x3C, 0x61, 
0x20, 0x68, 0x72, 0x65, 0x66, 0x3D, 0x22, 0x2F, 0x64, 0x61, 0x74, 0x61, 
0x2E, 0x73, 0x68, 0x74, 0x6D, 0x6C, 0x22, 0x3E, 0x47, 0x6F, 0x20, 0x74, 
0x6F, 0x20, 0x48, 0x6F, 0x6D, 0x65, 0x3C, 0x2F, 0x61, 0x3E, 0x0A, 0x20, 
0x20, 0x20, 0x3C, 0x2F, 0x62, 0x6F, 0x64, 0x79, 0x3E, 0x0A, 0x3C, 0x2F, 
0x68, 0x74, 0x6D, 0x6C, 0x3E, 0x0D, 0x0A, 0x0D, 0x0A};

#if FSDATA_FILE_ALIGNMENT==1
static const unsigned int dummy_align__data_html = 2;
#endif
static const unsigned char FSDATA_ALIGN_PRE data_html[] FSDATA_ALIGN_POST = {

// /data.shtml
0x2F, 0x64, 0x61, 0x74, 0x61, 0x2E, 0x73, 0x68, 0x74, 0x6D, 0x6C, 0x00,

// HTTP/1.0 200 OK
0x48, 0x54, 0x54, 0x50, 0x2F, 0x31, 0x2E, 0x30, 0x20, 0x32, 0x30, 0x30, 0x20, 0x4F, 0x4B, 0x0d,
0x0a,

/* "Server: lwIP/2.0.3d
" (64 bytes) */
0x53, 0x65, 0x72, 0x76, 0x65, 0x72, 0x3a, 0x20, 0x6c, 0x77, 0x49, 0x50, 0x2f, 0x32, 0x2e, 0x30,
0x2e, 0x33, 0x64, 0x0D, 0x0A,

/* "Content-Type: text/html" (27 bytes) */
0x43, 0x6f, 0x6e, 0x74, 0x65, 0x6e, 0x74, 0x2d, 0x54, 0x79, 0x70, 0x65, 0x3a, 0x20, 0x74, 0x65,
0x78, 0x74, 0x2f, 0x68, 0x74, 0x6d, 0x6c, 0x0d, 0x0a, 0x0d, 0x0a,

0x3C, 0x21, 0x44, 0x4F, 0x43, 0x54, 0x59, 0x50, 0x45, 0x20, 0x68, 0x74, 
0x6D, 0x6C, 0x3E, 0x0A, 0x3C, 0x68, 0x74, 0x6D, 0x6C, 0x3E, 0x0A, 0x20, 0x20, 
0x20, 0x20, 0x3C, 0x68, 0x65, 0x61, 0x64, 0x3E, 0x20, 0x0A, 0x20, 0x20, 
0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x3C, 0x74, 0x69, 0x74, 0x6C, 0x65, 
0x3E, 0x49, 0x6E, 0x66, 0x6F, 0x72, 0x6D, 0x61, 0x74, 0x69, 0x6F, 0x6E, 
0x20, 0x47, 0x61, 0x74, 0x68, 0x65, 0x72, 0x69, 0x6E, 0x67, 0x3C, 0x2F, 
0x74, 0x69, 0x74, 0x6C, 0x65, 0x3E, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x3C, 
0x2F, 0x68, 0x65, 0x61, 0x64, 0x3E, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x3C, 
0x62, 0x6F, 0x64, 0x79, 0x3E, 0x20, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 
0x20, 0x20, 0x20, 0x3C, 0x68, 0x31, 0x3E, 0x44, 0x61, 0x74, 0x61, 0x20, 
0x43, 0x6F, 0x6C, 0x6C, 0x65, 0x63, 0x74, 0x65, 0x64, 0x3C, 0x2F, 0x68, 
0x31, 0x3E, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x3C, 
0x70, 0x3E, 0x3C, 0x21, 0x2D, 0x2D, 0x23, 0x70, 0x65, 0x6F, 0x70, 0x6C, 
0x65, 0x2D, 0x2D, 0x3E, 0x3C, 0x2F, 0x70, 0x3E, 0x0A, 0x20, 0x20, 0x20, 
0x20, 0x3C, 0x2F, 0x62, 0x6F, 0x64, 0x79, 0x3E, 0x0A, 0x0A, 0x3C, 0x2F, 
0x68, 0x74, 0x6D, 0x6C, 0x3E, 0x0A, 0x0D, 0x0A,
};

#if FSDATA_FILE_ALIGNMENT==1
static const unsigned int dummy_align__chart_html = 3;
#endif
static const unsigned char FSDATA_ALIGN_PRE chart_html[] FSDATA_ALIGN_POST = {

// /chart.shmtl
0x2F, 0x63, 0x68, 0x61, 0x72, 0x74, 0x2E, 0x73, 0x68, 0x74, 0x6D, 0x6C, 0x00,

// HTTP/1.0 200 OK
0x48, 0x54, 0x54, 0x50, 0x2F, 0x31, 0x2E, 0x30, 0x20, 0x32, 0x30, 0x30, 0x20, 0x4F, 0x4B, 0x0d,
0x0a,

/* "Server: lwIP/2.0.3d
" (64 bytes) */
0x53, 0x65, 0x72, 0x76, 0x65, 0x72, 0x3a, 0x20, 0x6c, 0x77, 0x49, 0x50, 0x2f, 0x32, 0x2e, 0x30,
0x2e, 0x33, 0x64, 0x0D, 0x0A,

/* "Content-Type: text/html" (27 bytes) */
0x43, 0x6f, 0x6e, 0x74, 0x65, 0x6e, 0x74, 0x2d, 0x54, 0x79, 0x70, 0x65, 0x3a, 0x20, 0x74, 0x65,
0x78, 0x74, 0x2f, 0x68, 0x74, 0x6d, 0x6c, 0x0d, 0x0a, 0x0d, 0x0a,

// Raw-data
0x3C, 0x21, 0x44, 0x4F, 0x43, 0x54, 0x59, 0x50, 0x45, 0x20, 0x68, 0x74, 
0x6D, 0x6C, 0x3E, 0x0A, 0x3C, 0x68, 0x74, 0x6D, 0x6C, 0x3E, 0x0A, 0x3C, 
0x68, 0x65, 0x61, 0x64, 0x3E, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x3C, 0x74, 
0x69, 0x74, 0x6C, 0x65, 0x3E, 0x53, 0x69, 0x67, 0x6E, 0x61, 0x6C, 0x20, 
0x44, 0x61, 0x73, 0x68, 0x62, 0x6F, 0x61, 0x72, 0x64, 0x3C, 0x2F, 0x74, 
0x69, 0x74, 0x6C, 0x65, 0x3E, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x3C, 0x73, 
0x63, 0x72, 0x69, 0x70, 0x74, 0x20, 0x73, 0x72, 0x63, 0x3D, 0x22, 0x68, 
0x74, 0x74, 0x70, 0x73, 0x3A, 0x2F, 0x2F, 0x63, 0x6F, 0x64, 0x65, 0x2E, 
0x6A, 0x71, 0x75, 0x65, 0x72, 0x79, 0x2E, 0x63, 0x6F, 0x6D, 0x2F, 0x6A, 
0x71, 0x75, 0x65, 0x72, 0x79, 0x2D, 0x33, 0x2E, 0x36, 0x2E, 0x30, 0x2E, 
0x6D, 0x69, 0x6E, 0x2E, 0x6A, 0x73, 0x22, 0x3E, 0x3C, 0x2F, 0x73, 0x63, 
0x72, 0x69, 0x70, 0x74, 0x3E, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x3C, 0x73, 
0x63, 0x72, 0x69, 0x70, 0x74, 0x20, 0x73, 0x72, 0x63, 0x3D, 0x22, 0x68, 
0x74, 0x74, 0x70, 0x73, 0x3A, 0x2F, 0x2F, 0x63, 0x64, 0x6E, 0x2E, 0x6A, 
0x73, 0x64, 0x65, 0x6C, 0x69, 0x76, 0x72, 0x2E, 0x6E, 0x65, 0x74, 0x2F, 
0x6E, 0x70, 0x6D, 0x2F, 0x63, 0x68, 0x61, 0x72, 0x74, 0x2E, 0x6A, 0x73, 
0x22, 0x3E, 0x3C, 0x2F, 0x73, 0x63, 0x72, 0x69, 0x70, 0x74, 0x3E, 0x0A, 
0x3C, 0x2F, 0x68, 0x65, 0x61, 0x64, 0x3E, 0x0A, 0x3C, 0x62, 0x6F, 0x64, 
0x79, 0x3E, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x3C, 0x63, 0x61, 0x6E, 0x76, 
0x61, 0x73, 0x20, 0x69, 0x64, 0x3D, 0x22, 0x73, 0x69, 0x67, 0x6E, 0x61, 
0x6C, 0x43, 0x68, 0x61, 0x72, 0x74, 0x22, 0x20, 0x77, 0x69, 0x64, 0x74, 
0x68, 0x3D, 0x22, 0x34, 0x30, 0x30, 0x22, 0x20, 0x68, 0x65, 0x69, 0x67, 
0x68, 0x74, 0x3D, 0x22, 0x32, 0x30, 0x30, 0x22, 0x3E, 0x3C, 0x2F, 0x63, 
0x61, 0x6E, 0x76, 0x61, 0x73, 0x3E, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x3C, 
0x73, 0x63, 0x72, 0x69, 0x70, 0x74, 0x3E, 0x0A, 0x20, 0x20, 0x20, 0x20, 
0x20, 0x20, 0x20, 0x20, 0x63, 0x6F, 0x6E, 0x73, 0x74, 0x20, 0x63, 0x74, 
0x78, 0x20, 0x3D, 0x20, 0x64, 0x6F, 0x63, 0x75, 0x6D, 0x65, 0x6E, 0x74, 
0x2E, 0x67, 0x65, 0x74, 0x45, 0x6C, 0x65, 0x6D, 0x65, 0x6E, 0x74, 0x42, 
0x79, 0x49, 0x64, 0x28, 0x27, 0x73, 0x69, 0x67, 0x6E, 0x61, 0x6C, 0x43, 
0x68, 0x61, 0x72, 0x74, 0x27, 0x29, 0x2E, 0x67, 0x65, 0x74, 0x43, 0x6F, 
0x6E, 0x74, 0x65, 0x78, 0x74, 0x28, 0x27, 0x32, 0x64, 0x27, 0x29, 0x3B, 
0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x63, 0x6F, 0x6E, 
0x73, 0x74, 0x20, 0x73, 0x69, 0x67, 0x6E, 0x61, 0x6C, 0x43, 0x68, 0x61, 
0x72, 0x74, 0x20, 0x3D, 0x20, 0x6E, 0x65, 0x77, 0x20, 0x43, 0x68, 0x61, 
0x72, 0x74, 0x28, 0x63, 0x74, 0x78, 0x2C, 0x20, 0x7B, 0x0A, 0x20, 0x20, 
0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x74, 0x79, 
0x70, 0x65, 0x3A, 0x20, 0x27, 0x6C, 0x69, 0x6E, 0x65, 0x27, 0x2C, 0x0A, 
0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 
0x64, 0x61, 0x74, 0x61, 0x3A, 0x20, 0x7B, 0x0A, 0x20, 0x20, 0x20, 0x20, 
0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 
0x6C, 0x61, 0x62, 0x65, 0x6C, 0x73, 0x3A, 0x20, 0x5B, 0x5D, 0x2C, 0x20, 
0x2F, 0x2F, 0x20, 0x54, 0x69, 0x6D, 0x65, 0x20, 0x6F, 0x72, 0x20, 0x78, 
0x2D, 0x61, 0x78, 0x69, 0x73, 0x20, 0x6C, 0x61, 0x62, 0x65, 0x6C, 0x73, 
0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 
0x20, 0x20, 0x20, 0x20, 0x20, 0x64, 0x61, 0x74, 0x61, 0x73, 0x65, 0x74, 
0x73, 0x3A, 0x20, 0x5B, 0x7B, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 
0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 
0x20, 0x20, 0x6C, 0x61, 0x62, 0x65, 0x6C, 0x3A, 0x20, 0x27, 0x53, 0x69, 
0x67, 0x6E, 0x61, 0x6C, 0x20, 0x56, 0x61, 0x6C, 0x75, 0x65, 0x27, 0x2C, 
0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 
0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x64, 0x61, 0x74, 
0x61, 0x3A, 0x20, 0x5B, 0x5D, 0x2C, 0x20, 0x2F, 0x2F, 0x20, 0x59, 0x2D, 
0x61, 0x78, 0x69, 0x73, 0x20, 0x76, 0x61, 0x6C, 0x75, 0x65, 0x73, 0x0A, 
0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 
0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x62, 0x6F, 0x72, 0x64, 
0x65, 0x72, 0x43, 0x6F, 0x6C, 0x6F, 0x72, 0x3A, 0x20, 0x27, 0x72, 0x67, 
0x62, 0x61, 0x28, 0x37, 0x35, 0x2C, 0x20, 0x31, 0x39, 0x32, 0x2C, 0x20, 
0x31, 0x39, 0x32, 0x2C, 0x20, 0x31, 0x29, 0x27, 0x2C, 0x0A, 0x20, 0x20, 
0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 
0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x62, 0x6F, 0x72, 0x64, 0x65, 0x72, 
0x57, 0x69, 0x64, 0x74, 0x68, 0x3A, 0x20, 0x31, 0x2C, 0x0A, 0x20, 0x20, 
0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 
0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x66, 0x69, 0x6C, 0x6C, 0x3A, 0x20, 
0x66, 0x61, 0x6C, 0x73, 0x65, 0x2C, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 
0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x7D, 
0x5D, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 
0x20, 0x20, 0x7D, 0x2C, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 
0x20, 0x20, 0x20, 0x20, 0x20, 0x6F, 0x70, 0x74, 0x69, 0x6F, 0x6E, 0x73, 
0x3A, 0x20, 0x7B, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 
0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x73, 0x63, 0x61, 0x6C, 
0x65, 0x73, 0x3A, 0x20, 0x7B, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 
0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 
0x20, 0x20, 0x78, 0x3A, 0x20, 0x7B, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 
0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 
0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x74, 0x79, 0x70, 0x65, 0x3A, 
0x20, 0x27, 0x6C, 0x69, 0x6E, 0x65, 0x61, 0x72, 0x27, 0x2C, 0x0A, 0x20, 
0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 
0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x70, 
0x6F, 0x73, 0x69, 0x74, 0x69, 0x6F, 0x6E, 0x3A, 0x20, 0x27, 0x62, 0x6F, 
0x74, 0x74, 0x6F, 0x6D, 0x27, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 
0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 
0x20, 0x20, 0x7D, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 
0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x7D, 0x0A, 0x20, 0x20, 
0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x7D, 0x0A, 
0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x7D, 0x29, 0x3B, 0x0A, 
0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x66, 0x75, 0x6E, 
0x63, 0x74, 0x69, 0x6F, 0x6E, 0x20, 0x75, 0x70, 0x64, 0x61, 0x74, 0x65, 
0x43, 0x68, 0x61, 0x72, 0x74, 0x28, 0x29, 0x20, 0x7B, 0x0A, 0x20, 0x20, 
0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x24, 0x2E, 
0x61, 0x6A, 0x61, 0x78, 0x28, 0x7B, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 
0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x75, 
0x72, 0x6C, 0x3A, 0x20, 0x27, 0x2F, 0x64, 0x61, 0x74, 0x61, 0x27, 0x2C, 
0x20, 0x2F, 0x2F, 0x20, 0x45, 0x6E, 0x64, 0x70, 0x6F, 0x69, 0x6E, 0x74, 
0x20, 0x74, 0x6F, 0x20, 0x67, 0x65, 0x74, 0x20, 0x74, 0x68, 0x65, 0x20, 
0x6C, 0x61, 0x74, 0x65, 0x73, 0x74, 0x20, 0x64, 0x61, 0x74, 0x61, 0x0A, 
0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 
0x20, 0x20, 0x20, 0x20, 0x6D, 0x65, 0x74, 0x68, 0x6F, 0x64, 0x3A, 0x20, 
0x27, 0x47, 0x45, 0x54, 0x27, 0x2C, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 
0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x73, 
0x75, 0x63, 0x63, 0x65, 0x73, 0x73, 0x3A, 0x20, 0x66, 0x75, 0x6E, 0x63, 
0x74, 0x69, 0x6F, 0x6E, 0x28, 0x64, 0x61, 0x74, 0x61, 0x29, 0x20, 0x7B, 
0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 
0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x63, 0x6F, 0x6E, 
0x73, 0x74, 0x20, 0x73, 0x69, 0x67, 0x6E, 0x61, 0x6C, 0x56, 0x61, 0x6C, 
0x75, 0x65, 0x20, 0x3D, 0x20, 0x64, 0x61, 0x74, 0x61, 0x2E, 0x73, 0x69, 
0x67, 0x6E, 0x61, 0x6C, 0x3B, 0x20, 0x2F, 0x2F, 0x20, 0x41, 0x64, 0x6A, 
0x75, 0x73, 0x74, 0x20, 0x62, 0x61, 0x73, 0x65, 0x64, 0x20, 0x6F, 0x6E, 
0x20, 0x79, 0x6F, 0x75, 0x72, 0x20, 0x73, 0x65, 0x72, 0x76, 0x65, 0x72, 
0x27, 0x73, 0x20, 0x72, 0x65, 0x73, 0x70, 0x6F, 0x6E, 0x73, 0x65, 0x20, 
0x66, 0x6F, 0x72, 0x6D, 0x61, 0x74, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 
0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 
0x20, 0x20, 0x20, 0x63, 0x6F, 0x6E, 0x73, 0x74, 0x20, 0x74, 0x69, 0x6D, 
0x65, 0x73, 0x74, 0x61, 0x6D, 0x70, 0x20, 0x3D, 0x20, 0x6E, 0x65, 0x77, 
0x20, 0x44, 0x61, 0x74, 0x65, 0x28, 0x29, 0x2E, 0x74, 0x6F, 0x4C, 0x6F, 
0x63, 0x61, 0x6C, 0x65, 0x54, 0x69, 0x6D, 0x65, 0x53, 0x74, 0x72, 0x69, 
0x6E, 0x67, 0x28, 0x29, 0x3B, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 
0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 
0x20, 0x20, 0x73, 0x69, 0x67, 0x6E, 0x61, 0x6C, 0x43, 0x68, 0x61, 0x72, 
0x74, 0x2E, 0x64, 0x61, 0x74, 0x61, 0x2E, 0x6C, 0x61, 0x62, 0x65, 0x6C, 
0x73, 0x2E, 0x70, 0x75, 0x73, 0x68, 0x28, 0x74, 0x69, 0x6D, 0x65, 0x73, 
0x74, 0x61, 0x6D, 0x70, 0x29, 0x3B, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 
0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 
0x20, 0x20, 0x20, 0x73, 0x69, 0x67, 0x6E, 0x61, 0x6C, 0x43, 0x68, 0x61, 
0x72, 0x74, 0x2E, 0x64, 0x61, 0x74, 0x61, 0x2E, 0x64, 0x61, 0x74, 0x61, 
0x73, 0x65, 0x74, 0x73, 0x5B, 0x30, 0x5D, 0x2E, 0x64, 0x61, 0x74, 0x61, 
0x2E, 0x70, 0x75, 0x73, 0x68, 0x28, 0x73, 0x69, 0x67, 0x6E, 0x61, 0x6C, 
0x56, 0x61, 0x6C, 0x75, 0x65, 0x29, 0x3B, 0x0A, 0x0A, 0x20, 0x20, 0x20, 
0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 
0x20, 0x20, 0x20, 0x20, 0x20, 0x2F, 0x2F, 0x20, 0x4C, 0x69, 0x6D, 0x69, 
0x74, 0x20, 0x74, 0x68, 0x65, 0x20, 0x6E, 0x75, 0x6D, 0x62, 0x65, 0x72, 
0x20, 0x6F, 0x66, 0x20, 0x70, 0x6F, 0x69, 0x6E, 0x74, 0x73, 0x20, 0x6F, 
0x6E, 0x20, 0x74, 0x68, 0x65, 0x20, 0x63, 0x68, 0x61, 0x72, 0x74, 0x0A, 
0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 
0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x69, 0x66, 0x20, 0x28, 
0x73, 0x69, 0x67, 0x6E, 0x61, 0x6C, 0x43, 0x68, 0x61, 0x72, 0x74, 0x2E, 
0x64, 0x61, 0x74, 0x61, 0x2E, 0x6C, 0x61, 0x62, 0x65, 0x6C, 0x73, 0x2E, 
0x6C, 0x65, 0x6E, 0x67, 0x74, 0x68, 0x20, 0x3E, 0x20, 0x32, 0x30, 0x29, 
0x20, 0x7B, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 
0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 
0x20, 0x20, 0x20, 0x73, 0x69, 0x67, 0x6E, 0x61, 0x6C, 0x43, 0x68, 0x61, 
0x72, 0x74, 0x2E, 0x64, 0x61, 0x74, 0x61, 0x2E, 0x6C, 0x61, 0x62, 0x65, 
0x6C, 0x73, 0x2E, 0x73, 0x68, 0x69, 0x66, 0x74, 0x28, 0x29, 0x3B, 0x0A, 
0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 
0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 
0x73, 0x69, 0x67, 0x6E, 0x61, 0x6C, 0x43, 0x68, 0x61, 0x72, 0x74, 0x2E, 
0x64, 0x61, 0x74, 0x61, 0x2E, 0x64, 0x61, 0x74, 0x61, 0x73, 0x65, 0x74, 
0x73, 0x5B, 0x30, 0x5D, 0x2E, 0x64, 0x61, 0x74, 0x61, 0x2E, 0x73, 0x68, 
0x69, 0x66, 0x74, 0x28, 0x29, 0x3B, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 
0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 
0x20, 0x20, 0x20, 0x7D, 0x0A, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 
0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 
0x20, 0x20, 0x73, 0x69, 0x67, 0x6E, 0x61, 0x6C, 0x43, 0x68, 0x61, 0x72, 
0x74, 0x2E, 0x75, 0x70, 0x64, 0x61, 0x74, 0x65, 0x28, 0x29, 0x3B, 0x0A, 
0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 
0x20, 0x20, 0x20, 0x20, 0x7D, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 
0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x7D, 0x29, 0x3B, 0x0A, 0x20, 0x20, 
0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x7D, 0x0A, 0x0A, 0x20, 0x20, 0x20, 
0x20, 0x20, 0x20, 0x20, 0x20, 0x2F, 0x2F, 0x20, 0x52, 0x65, 0x66, 0x72, 
0x65, 0x73, 0x68, 0x20, 0x64, 0x61, 0x74, 0x61, 0x20, 0x65, 0x76, 0x65, 
0x72, 0x79, 0x20, 0x73, 0x65, 0x63, 0x6F, 0x6E, 0x64, 0x0A, 0x20, 0x20, 
0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x73, 0x65, 0x74, 0x49, 0x6E, 0x74, 
0x65, 0x72, 0x76, 0x61, 0x6C, 0x28, 0x75, 0x70, 0x64, 0x61, 0x74, 0x65, 
0x43, 0x68, 0x61, 0x72, 0x74, 0x2C, 0x20, 0x31, 0x30, 0x30, 0x30, 0x29, 
0x3B, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x3C, 0x2F, 0x73, 0x63, 0x72, 0x69, 
0x70, 0x74, 0x3E, 0x0A, 0x3C, 0x2F, 0x62, 0x6F, 0x64, 0x79, 0x3E, 0x0A, 
0x3C, 0x2F, 0x68, 0x74, 0x6D, 0x6C, 0x3E,
};

const struct fsdata_file file__404_html[] = { {
file_NULL,
page_404_html,
page_404_html + 12,
sizeof(page_404_html) - 12,
FS_FILE_FLAGS_HEADER_INCLUDED | FS_FILE_FLAGS_HEADER_PERSISTENT,
}};

const struct fsdata_file file__data_html[] = { {
file__404_html,
data_html,
data_html + 12,
sizeof(data_html) - 12,
FS_FILE_FLAGS_HEADER_INCLUDED | FS_FILE_FLAGS_HEADER_PERSISTENT,
}};

const struct fsdata_file file__chart_html[] = { {
file__data_html,
chart_html,
chart_html + 13,
sizeof(chart_html) - 13,
FS_FILE_FLAGS_HEADER_INCLUDED | FS_FILE_FLAGS_HEADER_PERSISTENT,
}};

const struct fsdata_file file__index_html[] = { {
file__chart_html,
index_html,
index_html + 13,
sizeof(index_html) - 13,
FS_FILE_FLAGS_HEADER_INCLUDED | FS_FILE_FLAGS_HEADER_PERSISTENT,
}};

#define FS_ROOT file__index_html
#define FS_NUMFILES 4