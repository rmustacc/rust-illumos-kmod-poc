#![crate_type="staticlib"]
#![feature(extern_types)]
#![feature(lang_items)]
#![no_std]

use core::panic::PanicInfo;

const CE_WARN: i32 = 2;
const MODREV_1: i32 = 1;
const RMOD_STR: *const u8 = b"rust module\0".as_ptr();
const DEVO_REV: i32 = 4;
const DDI_SUCCESS : i32 = 0;
const DDI_FAILURE : i32 = -1;
const DDI_ATTACH : i32 = 0;
const DDI_RESUME : i32 = 1;
const DDI_DETACH: i32 = 0;
const DDI_SUSPEND: i32 = 1;

const DDI_DEVICE_ATTR_V1 : u16 = 2;
const DDI_STRUCTURE_LE_ACC : u8 = 1;
const DDI_STRICTORDER_ACC : u8 = 1;
const DDI_DEFAULT_ACC : u8 = 1;

const KM_SLEEP : i32 = 0;

const SENSOR_UNIT_CELSIUS : u32 = 1;

/*
 * The pchtemp registers are in Bar 1.
 */
const PCHTEMP_RNUMBER : u32 = 1;
const PCHTEMP_TEMP_RESOLUTION : i32 = 2;
const PCHTEMP_TEMP_OFFSET : i64 = 50 << 1;

extern {
    type ModOps;
    type CbOps;
    type BusOps;
    type DevInfo;
    type ModInfo;
    type DdiAccHandle;

	fn __dtrace_probe_rust();
    fn cmn_err(code: i32, msg: *const u8, ...);
    fn nulldev() -> i32;
    fn nodev() -> i32;
    fn ddi_quiesce_not_needed() -> i32;
    fn mod_install(linkage: * const IllumosModLinkage) -> i32;
    fn mod_remove(linkage: * const IllumosModLinkage) -> i32;
    fn mod_info(linkage: * const IllumosModLinkage, info: *const ModInfo) -> i32;
    fn ddi_dev_regsize(dip: * const DevInfo, rnumber: u32, lenp: * mut i64) -> i32;
    fn ddi_regs_map_setup(dip: * const DevInfo, rnumber: u32, addrp: * mut u64,
       offset: i64, len: i64, attrp: * const IllumosDDIDevAccAttr,
       handlep : * mut * mut DdiAccHandle) -> i32;
    fn ddi_regs_map_free(handle: * mut * mut DdiAccHandle);
    fn ddi_get8(handle: * mut DdiAccHandle, dev_addr: * const u8) -> u8;
    fn ddi_get16(handle: * mut DdiAccHandle, dev_addr: * const u16) -> u16;
    fn ddi_set_driver_private(dip: * mut DevInfo, datap : * mut u8);
    fn ddi_get_driver_private(dip: * mut DevInfo) -> * mut u8;
    fn kmem_zalloc(size : usize, flag : i32) -> * mut u8;
    fn kmem_free(data : * mut u8, size: usize);

    fn ksensor_kind_temperature(data : * mut u8, kind : * mut SensorIoctlKind) -> i32;
    fn ksensor_create(dip: * mut DevInfo, ops: * const KsensorOps, arg: * mut u8,
        name: * const u8, class: * const u8, id: * mut i32) -> i32;
    fn ksensor_remove_sensor(dip: * mut DevInfo, id: i32);

    static mod_driverops: ModOps;
}

#[repr(C)]
struct IllumosDDIDevAccAttr {
    devacc_attr_version: u16,
    devacc_attr_endian_flags: u8,
    devacc_attr_dataorder: u8,
    devacc_attr_access: u8
}

#[repr(C)]
struct IllumosDevOps {
    devo_rev: i32,
    devo_refcnt: i32,
    devo_getinfo: unsafe extern "C" fn () -> i32,
    devo_identify: unsafe extern "C" fn () -> i32,
    devo_probe: unsafe extern "C" fn () -> i32,
    devo_attach: unsafe extern "C" fn (* mut DevInfo, i32) -> i32,
    devo_detach: unsafe extern "C" fn (* mut DevInfo, i32) -> i32,
    devo_reset: unsafe extern "C" fn () -> i32,
    devo_cb_ops: *const CbOps,
    devo_bus_ops: *const BusOps,
    devo_power: unsafe extern "C" fn () -> i32,
    devo_quiesce: unsafe extern "C" fn () -> i32,
}
unsafe impl Sync for IllumosDevOps {}

static RMOD_DEVOPS : IllumosDevOps = IllumosDevOps {
    devo_rev: DEVO_REV,
    devo_refcnt: 0,
    devo_getinfo: nodev,
    devo_identify: nulldev,
    devo_probe: nulldev,
    devo_attach: rmod_attach,
    devo_detach: rmod_detach,
    devo_reset: nodev,
    devo_cb_ops: 0 as *const CbOps,
    devo_bus_ops: 0 as *const BusOps,
    devo_power: nodev,
    devo_quiesce: ddi_quiesce_not_needed
};

#[repr(C)]
struct IllumosModLdrv {
    drv_modops: *const ModOps,
    drv_linkinfo: *const u8,
    drv_ops: *const IllumosDevOps,
}
unsafe impl Sync for IllumosModLdrv {}

static mut RMOD_MODLDRV : IllumosModLdrv = unsafe {
    IllumosModLdrv {
        drv_modops: &mod_driverops,
        drv_linkinfo: RMOD_STR,
        drv_ops: &RMOD_DEVOPS as * const IllumosDevOps,
    }
};

#[repr(C)]
struct IllumosModLinkage {
    ml_rev: i32,
    ml_linkage: [* const IllumosModLdrv; 7]
}
unsafe impl Sync for IllumosModLinkage {}

#[no_mangle]
static RMOD_MODLINKAGE: IllumosModLinkage = unsafe {
    IllumosModLinkage {
        ml_rev: MODREV_1,
        ml_linkage: [ &RMOD_MODLDRV as * const IllumosModLdrv,
              0 as * mut IllumosModLdrv, 0 as * mut IllumosModLdrv, 
              0 as * mut IllumosModLdrv, 0 as * mut IllumosModLdrv,
              0 as * mut IllumosModLdrv, 0 as * mut IllumosModLdrv ]
    }
};

#[repr(C)]
struct SensorIoctlKind {
    sik_kind: u64
}

#[repr(C)]
struct SensorIoctlTemperature {
    sit_unit: u32,
    sit_gran: i32,
    sit_prec: u32,
    sit_pad: u32,
    sit_temp: i64
}

#[repr(C)]
struct KsensorOps {
    kso_kind: unsafe extern "C" fn (* mut u8, * mut SensorIoctlKind) -> i32,
    kso_temp: unsafe extern "C" fn (* mut u8, * mut SensorIoctlTemperature) -> i32,
    kso_label: Option<unsafe extern "C" fn () -> i32>,
}

struct TempData {
    td_devinfo: *mut DevInfo,
    td_regp: *mut DdiAccHandle,
    td_regbase : u64,
    td_id: i32,
    td_tempreg : u16,
    td_tselreg : u8,
    td_temp: i64
}

static RMOD_TEMP_OPS : KsensorOps  = KsensorOps {
    kso_kind: ksensor_kind_temperature,
    kso_temp: rmod_gettemp,
    kso_label: None
};

unsafe extern "C" fn rmod_gettemp(arg : * mut u8, tempp : * mut SensorIoctlTemperature) -> i32 {
    let mut td : *mut TempData = arg as * mut TempData;
    let tsel_reg : u64 = (*td).td_regbase + 8;

    (*td).td_tempreg = ddi_get16((*td).td_regp, (*td).td_regbase as * const u16);
    (*td).td_tselreg = ddi_get8((*td).td_regp, tsel_reg as * const u8);
    if ((*td).td_tselreg & 0x1) == 0 {
        return 6
    }

    __dtrace_probe_rust();

    (*td).td_temp = ((*td).td_tempreg as i64) - PCHTEMP_TEMP_OFFSET;
   
    (*tempp).sit_unit = SENSOR_UNIT_CELSIUS;
    (*tempp).sit_gran = PCHTEMP_TEMP_RESOLUTION;
    (*tempp).sit_prec = 0;
    (*tempp).sit_temp = (*td).td_temp;
    0
}

fn rmod_cleanup(td: * mut TempData) {
    unsafe {
        ddi_regs_map_free(&mut (*td).td_regp);
        ksensor_remove_sensor((*td).td_devinfo, (*td).td_id);
        kmem_free(td as * mut u8, core::mem::size_of::<TempData>());
    }
}

#[no_mangle]
unsafe extern "C" fn rmod_attach(dip : * mut DevInfo, cmd : i32) -> i32 {
    if cmd == DDI_RESUME {
         return DDI_SUCCESS;
    } else if cmd != DDI_ATTACH {
         return DDI_FAILURE;
    }

    let mut reglen : i64 = 0;
    let ret = ddi_dev_regsize(dip, PCHTEMP_RNUMBER, &mut reglen);
    if ret != DDI_SUCCESS {
        cmn_err(CE_WARN, b"failed to get register size\0".as_ptr());
        return DDI_FAILURE;
    }

    let attr = IllumosDDIDevAccAttr {
        devacc_attr_version: DDI_DEVICE_ATTR_V1,
        devacc_attr_endian_flags: DDI_STRUCTURE_LE_ACC,
        devacc_attr_dataorder: DDI_STRICTORDER_ACC,
        devacc_attr_access: DDI_DEFAULT_ACC 
    };

    let kmem : * mut u8 = kmem_zalloc(core::mem::size_of::<TempData>(), KM_SLEEP);
    let mut addr: u64 = 0;
    let mut handle : * mut DdiAccHandle = 0 as * mut DdiAccHandle;
    let ret = ddi_regs_map_setup(dip, PCHTEMP_RNUMBER, &mut addr, 0, reglen,
        &attr, &mut handle);
    if ret != DDI_SUCCESS {
        cmn_err(CE_WARN, b"regs map setup failed\0".as_ptr());
    }

    let mut td : *mut TempData = kmem as * mut TempData;
    (*td).td_devinfo = dip;
    (*td).td_regp = handle;
    (*td).td_regbase = addr;
    (*td).td_temp = 0;

    let regval : u16 = ddi_get16(handle, addr as * const u16);
    cmn_err(CE_WARN, b"read temp reg: 0x%x\0".as_ptr(), regval as u32);

    let ret = ksensor_create(dip, &RMOD_TEMP_OPS as * const KsensorOps, kmem,
        b"ts.0\0".as_ptr(), b"ddi_sensor:temperature:pch\0".as_ptr(),
        &mut (*td).td_id);
    if ret != 0 {
        cmn_err(CE_WARN, b"failed to make the sensor, sorry, got %d\0".as_ptr(),
            ret);
        rmod_cleanup(td);
        return DDI_FAILURE;
    }

    ddi_set_driver_private(dip, kmem);
    DDI_SUCCESS
}

#[no_mangle]
unsafe extern "C" fn rmod_detach(dip : * mut DevInfo, cmd : i32) -> i32 {
    if cmd == DDI_SUSPEND {
        return DDI_SUCCESS;
    } else if cmd != DDI_DETACH {
        return DDI_FAILURE;
    }

    let data = ddi_get_driver_private(dip);
    if data == 0 as * mut u8 {
         cmn_err(CE_WARN, b"asked to detach, but no private data\0".as_ptr());
         return DDI_FAILURE;
    }

    rmod_cleanup(data as * mut TempData);
    DDI_SUCCESS
}

#[no_mangle]
unsafe extern "C" fn _init() -> i32 {
    return mod_install(&RMOD_MODLINKAGE as * const IllumosModLinkage)
}

#[no_mangle]
unsafe extern "C" fn _info(modinfop: *const ModInfo) -> i32 {
    return mod_info(&RMOD_MODLINKAGE as * const IllumosModLinkage, modinfop);
}

#[no_mangle]
unsafe extern "C" fn _fini() -> i32 {
    return mod_remove(&RMOD_MODLINKAGE as * const IllumosModLinkage)
}

#[lang = "eh_personality"]
extern fn eh_personality() {}
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}
