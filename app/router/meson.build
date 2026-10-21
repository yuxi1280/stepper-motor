project('router', 'c')

# meson setup build --cross-file hc32.txt
# meson setup build --reconfigure
# meson compile -C build
# meson compile -C build program

message(host_machine.system())
message(host_machine.cpu_family())
message(host_machine.cpu())
message(host_machine.endian())

message(find_program('c').full_path())

pyocd_target = 'hc32f4a0xi'

objcopy  = '@0@'.format(find_program('objcopy').full_path())
objdump  = '@0@'.format(find_program('objdump').full_path())


lib_dir = 'D:/programs/arm-gnu-toolchain-12.2/lib/gcc/arm-none-eabi/12.2.0/thumb/v7e-m+fp/hard'
lwip_dir = '../lwip-2.2.1'
driver_src_dir = '../HC32F4A0_DDL_Rev2.4.0/drivers/hc32_ll_driver/src'
usb_device_lib_dir = '../HC32F4A0_DDL_Rev2.4.0/midwares/hc32/usb/usb_device_lib'

link_script = 'HC32F4A0xI.ld'

c_defines = [
  '-DHC32F4A0',
  '-DUSE_DDL_DRIVER',
  '-D__TARGET_FPU_VFP',
  '-D__FPU_PRESENT',
  ]

c_args = [
  '-std=c99',
  '-mthumb',
  '-mcpu=cortex-m4',
  '-mfloat-abi=hard',
  '-mfpu=fpv4-sp-d16',
  '-Os',
  '-Wall',
  '-Werror',
  c_defines,
  ]

link_args = [
  c_args,
  '-LD:/programs/arm-gnu-toolchain-12.2/lib/gcc/arm-none-eabi/12.2.0/thumb/v7e-m+fp/hard',
  '-T', join_paths(meson.current_source_dir(), link_script),
  '-nostdlib',
  '-lc_nano',
  '-lgcc',
  meson.current_source_dir()+'/'+lwip_dir+'/lwip.a',
  ]


include_dirs = [
  '../HC32F4A0_DDL_Rev2.4.0/drivers/hc32_ll_driver/inc',
  '../HC32F4A0_DDL_Rev2.4.0/drivers/cmsis/Device/HDSC/hc32f4xx/Include',
  '../HC32F4A0_DDL_Rev2.4.0/drivers/cmsis/Include',
  '../HC32F4A0_DDL_Rev2.4.0/midwares/hc32/usb',
  '../HC32F4A0_DDL_Rev2.4.0/midwares/hc32/usb/usb_device_lib/device_core',
  lwip_dir,
  lwip_dir+'/src/include',
  lwip_dir+'/contrib/ports/hdsc/port/include',
  '.',
  ]


hc32_ll_srcs = [
  driver_src_dir+'/hc32_ll.c',
  driver_src_dir+'/hc32_ll_icg.c',  # hc32_ll_icg.c会将初始化参数固化至FLASH
  driver_src_dir+'/hc32_ll_clk.c',
  driver_src_dir+'/hc32_ll_pwc.c',
  driver_src_dir+'/hc32_ll_fcg.c',
  driver_src_dir+'/hc32_ll_efm.c',
  driver_src_dir+'/hc32_ll_sram.c',
  driver_src_dir+'/hc32_ll_interrupts.c',
  driver_src_dir+'/hc32_ll_utility.c',
  driver_src_dir+'/hc32_ll_gpio.c',
  driver_src_dir+'/hc32_ll_usart.c',
  driver_src_dir+'/hc32_ll_usb.c',
  driver_src_dir+'/hc32_ll_fcm.c',
  driver_src_dir+'/hc32_ll_rtc.c',
  driver_src_dir+'/hc32_ll_eth.c',
  driver_src_dir+'/hc32_ll_trng.c',
  driver_src_dir+'/hc32_ll_wdt.c',
  driver_src_dir+'/hc32_ll_rmu.c',
  driver_src_dir+'/hc32_ll_aos.c',
  driver_src_dir+'/hc32_ll_spi.c',
  driver_src_dir+'/hc32_ll_dma.c',
  ]

usb_srcs = [
  usb_device_lib_dir+'/device_core/usb_dev_core.c',
  usb_device_lib_dir+'/device_core/usb_dev_ctrleptrans.c',
  usb_device_lib_dir+'/device_core/usb_dev_driver.c',
  usb_device_lib_dir+'/device_core/usb_dev_int.c',
  usb_device_lib_dir+'/device_core/usb_dev_stdreq.c',
  'usb_dev_cdc_class.c',
  'usb_bsp.c',
  'usb_dev_desc.c',
  'usb_dev_user.c',
  ]

app_srcs = [
  'startup_hc32f4a0.S',
  'system_hc32f4a0.c',
  'newlib_sys_calls.c',
  'app.c',
  'board.c',
  'can.c',
  'can_context.c',
  'ring_buffer.c',
  'rtc.c',
  'board_rtl8201.c',
  'rtl8201.c',
  'ethernetif.c',
  'shelf.c',
  'shelf_usb.c',
  'esp_at_parser.c',
  'esp_at_report_callback.c',
  'new_memory_pool.c',
  ]

test_app_srcs = [
  driver_src_dir+'/hc32_ll.c',
  driver_src_dir+'/hc32_ll_icg.c',
  driver_src_dir+'/hc32_ll_gpio.c',
  'startup_hc32f4a0.S',
  'system_hc32f4a0.c',
  'test.c',
  ]


elf_file = 'test.elf'
hex_file = 'test.hex'


elf = executable(elf_file,
  [
    hc32_ll_srcs,
    usb_srcs,
    app_srcs,
  ],
  c_args: c_args,
  link_args: link_args,
  include_directories: include_dirs
  )

'''
elf = executable(elf_file,
  test_app_srcs,
  c_args: c_args,
  link_args: link_args,
  include_directories: include_dirs
  )
'''


custom_target('asm',
  capture: true,
  output: 'test.asm',
  build_by_default: false,
  command: [objdump, '-D', '-S', '-t', elf_file],
  depends: [elf]
  )

hex = custom_target('hex',
  output: hex_file,
  build_by_default: true,
  command: [objcopy, '-O', 'ihex', elf_file, '@OUTPUT@'],
  depends: [elf]
  )

custom_target('bin',
  output: 'test.bin',
  build_by_default: false,
  command: [objcopy, '-O', 'binary', elf_file, '@OUTPUT@'],
  depends: [elf]
  )

run_target('program',
  command: ['pyocd_program.bat', pyocd_target, hex_file],
  depends: [hex]
  )

