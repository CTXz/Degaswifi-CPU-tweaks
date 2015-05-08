/****************************************************************************
*
*    Copyright (c) 2005 - 2012 by Vivante Corp.
*    
*    This program is free software; you can redistribute it and/or modify
*    it under the terms of the GNU General Public License as published by
*    the Free Software Foundation; either version 2 of the license, or
*    (at your option) any later version.
*    
*    This program is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
*    GNU General Public License for more details.
*    
*    You should have received a copy of the GNU General Public License
*    along with this program; if not write to the Free Software
*    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*
*
*****************************************************************************/




#ifndef __gc_hal_kernel_device_h_
#define __gc_hal_kernel_device_h_

/******************************************************************************\
******************************* gckGALDEVICE Structure *******************************
\******************************************************************************/
typedef enum _gcePOWRE_MODE
{
    gcvPM_NORMAL,
#if MRVL_CONFIG_USE_PM_RUNTIME
    gcvPM_AUTO_SUSPEND,
#endif
    gcvPM_SUSPEND,
}
gcePOWER_MODE;


typedef struct _gckGALDEVICE
{
    /* Objects. */
    gckOS               os;
    gckKERNEL           kernels[gcdMAX_GPU_COUNT];
    struct device       *dev;

    /* Attributes. */
    gctSIZE_T           internalSize;
    gctPHYS_ADDR        internalPhysical;
    gctPOINTER          internalLogical;
    gckVIDMEM           internalVidMem;
    gctSIZE_T           externalSize;
    gctPHYS_ADDR        externalPhysical;
    gctPOINTER          externalLogical;
    gckVIDMEM           externalVidMem;
    gckVIDMEM           contiguousVidMem;
    gctPOINTER          contiguousBase;
    gctPHYS_ADDR        contiguousPhysical;
    gctSIZE_T           contiguousSize;
    gctBOOL             contiguousMapped;
    gctPOINTER          contiguousMappedUser;
    gctSIZE_T           systemMemorySize;
    gctUINT32           systemMemoryBaseAddress;
    gctPOINTER          registerBases[gcdMAX_GPU_COUNT];
    gctSIZE_T           registerSizes[gcdMAX_GPU_COUNT];
    gctUINT32           baseAddress;
    gctUINT32           requestedRegisterMemBases[gcdMAX_GPU_COUNT];
    gctSIZE_T           requestedRegisterMemSizes[gcdMAX_GPU_COUNT];
    gctUINT32           requestedContiguousBase;
    gctSIZE_T           requestedContiguousSize;

    /* IRQ management. */
    gctINT              irqLines[gcdMAX_GPU_COUNT];
    gctBOOL             isrInitializeds[gcdMAX_GPU_COUNT];
    gctBOOL             dataReadys[gcdMAX_GPU_COUNT];

    /* Thread management. */
    struct task_struct  *threadCtxts[gcdMAX_GPU_COUNT];
    struct semaphore    semas[gcdMAX_GPU_COUNT];
    gctBOOL             threadInitializeds[gcdMAX_GPU_COUNT];
    gctBOOL             killThread;

    gctBOOL             powerOffWhenIdle;
    gctBOOL             needPowerOff;
    gctUINT32           profileStep;
    gctUINT32           profileTimeSlice;
    gctUINT32           profileTailTimeSlice;
    gctUINT32           idleThreshold;

    /* debug flags. */
    gctBOOL             powerDebug;
    gctBOOL             pmrtDebug;
    gctBOOL             profilerDebug;
    gctBOOL             printPID;
    struct delayed_work pm_work;
    gctBOOL             is_work_inited;

    /* Signal management. */
    gctINT              signal;

    /* Core mapping */
    gceCORE             coreMapping[8];

    /* GC memory profile*/
    gctSIZE_T           reservedMem;
    gctINT32            vidMemUsage;
#if (MRVL_VIDEO_MEMORY_USE_TYPE != gcdMEM_TYPE_NONE)
    gctSIZE_T           reservedPmemMem;
    gctINT32            pmemUsage;
#endif
    gctINT32            contiguousNonPagedMemUsage;
    gctINT32            contiguousPagedMemUsage;
    gctINT32            virtualPagedMemUsage;
    gctINT32            wastBytes;
    /* GC video memory usage in detail. */
    gctINT32            indexVidMemUsage;
    gctINT32            vertexVidMemUsage;
    gctINT32            textureVidMemUsage;
    gctINT32            renderTargetVidMemUsage;
    gctINT32            depthVidMemUsage;
    gctINT32            bitmapVidMemUsage;
    gctINT32            tileStatusVidMemUsage;
    gctINT32            imageVidMemUsage;
    gctINT32            maskVidMemUsage;
    gctINT32            scissorVidMemUsage;
    gctINT32            hierarchicalDepthVidMemUsage;
    gctINT32            othersVidMemUsage;

    /* States before suspend. */
    gceCHIPPOWERSTATE   statesStored[gcdMAX_GPU_COUNT];

    /*Device Debug File System Entry in Kernel*/
   struct _gcsDebugFileSystemNode * dbgnode;

    /* current power mode */
    gcePOWER_MODE       currentPMode;
}
* gckGALDEVICE;

typedef struct _gcsHAL_PRIVATE_DATA
{
    gckGALDEVICE        device;
    gctPOINTER          mappedMemory;
    gctPOINTER          contiguousLogical;
    /* The process opening the device may not be the same as the one that closes it. */
    gctUINT32           pidOpen;
}
gcsHAL_PRIVATE_DATA, * gcsHAL_PRIVATE_DATA_PTR;

gceSTATUS gckGALDEVICE_Setup_ISR(
    IN gckGALDEVICE Device
    );

gceSTATUS gckGALDEVICE_Setup_ISR_2D(
    IN gckGALDEVICE Device
    );

gceSTATUS gckGALDEVICE_Setup_ISR_VG(
    IN gckGALDEVICE Device
    );

gceSTATUS gckGALDEVICE_Release_ISR(
    IN gckGALDEVICE Device
    );

gceSTATUS gckGALDEVICE_Release_ISR_2D(
    IN gckGALDEVICE Device
    );

gceSTATUS gckGALDEVICE_Release_ISR_VG(
    IN gckGALDEVICE Device
    );

gceSTATUS gckGALDEVICE_Start_Threads(
    IN gckGALDEVICE Device
    );

gceSTATUS gckGALDEVICE_Stop_Threads(
    gckGALDEVICE Device
    );

gceSTATUS gckGALDEVICE_Start(
    IN gckGALDEVICE Device
    );

gceSTATUS gckGALDEVICE_Stop(
    gckGALDEVICE Device
    );

gceSTATUS gckGALDEVICE_Construct(
    IN gctINT IrqLine,
    IN gctUINT32 RegisterMemBase,
    IN gctSIZE_T RegisterMemSize,
    IN gctINT IrqLine2D,
    IN gctUINT32 RegisterMemBase2D,
    IN gctSIZE_T RegisterMemSize2D,
    IN gctINT IrqLineVG,
    IN gctUINT32 RegisterMemBaseVG,
    IN gctSIZE_T RegisterMemSizeVG,
    IN gctUINT32 ContiguousBase,
    IN gctSIZE_T ContiguousSize,
#if (MRVL_VIDEO_MEMORY_USE_TYPE != gcdMEM_TYPE_NONE)
    IN gctSIZE_T PmemSize,
#endif
    IN gctSIZE_T BankSize,
    IN gctINT FastClear,
    IN gctINT Compression,
    IN gctUINT32 PhysBaseAddr,
    IN gctUINT32 PhysSize,
    IN gctINT Signal,
    IN gctUINT LogFileSize,
    IN gctINT PowerManagement,
    OUT gckGALDEVICE *Device
    );

gceSTATUS gckGALDEVICE_Destroy(
    IN gckGALDEVICE Device
    );

#endif /* __gc_hal_kernel_device_h_ */
