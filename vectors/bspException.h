#ifndef BSP_EXCEPTION_HANDLER_H
#define BSP_EXCEPTION_HANDLER_H
/* $Id$ */

/* A slightly improved exception 'default' exception handler for RTEMS / SVGM */

/* Author: Till Straumann <strauman@slac.stanford.edu>, 2002/5 */

#include <bsp/vectors.h>

#define BSP_EXCEPTION_NOTEPAD RTEMS_NOTEPAD_8

/* a user hook to be invoked before and after processing
 * the exception.
 * If the use hook returns a nonzero value, normal processing
 * is skipped (including the second call to the hook)
 * If the hook returns nonzero to the second call, no default
 * 'panic' occurs.
 * Default 'panic':
 *	- try to suspend interrupted task.
 *  - hang if no task context is available.
 */

typedef struct BSP_ExceptionExtensionRec_ *BSP_ExceptionExtension;

typedef int (*BSP_ExceptionHookProc)(BSP_Exception_frame *frame, BSP_ExceptionExtension ext, int after);

typedef struct BSP_ExceptionExtensionRec_ {
	BSP_ExceptionHookProc	hook;
	/* user fields may be added after this */
} BSP_ExceptionExtensionRec;

#define SRR1_TEA_EXC	(1<<(31-13))
#define SRR1_MCP_EXC	(1<<(31-12))

void
BSP_exceptionHandler(BSP_Exception_frame* excPtr);

#endif
