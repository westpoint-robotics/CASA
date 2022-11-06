#
#  Headless server image for cloud and device deployments
#
#  See README.md for documentation
#

/////////////// DEPLOYMENT //////////////////

source "vmware-iso" "casa_gold_headless" {
  // Docs: https://www.packer.io/plugins/builders/vmware/iso

  // VM Info:
  vm_name       = "casa_gold_headless_0x64"
  guest_os_type = "ubuntu64Guest"
  version       = "16"
  headless      = false
  // Virtual Hardware Specs
  memory       = 8172
  cpus         = 1
  cores        = 4
  disk_size    = 32000
  sound        = true
  disk_type_id = 0

  // ISO Details
  iso_urls = [    
    "https://cdimage.ubuntu.com/ubuntu-server/jammy/daily-live/20221030/jammy-live-server-amd64.iso"]
  iso_checksum     = "sha256:41fe67715e82ebf7b1ef10700e5a9cccd7db29079aab3377ee2878cf288378b2"
  iso_target_path  = var.local_iso_folder
  output_directory = var.output_directory
  snapshot_name    = "clean"
  http_directory   = "http"
  ssh_username     = "analyst"
  ssh_password     = var.password
  shutdown_command = "sudo shutdown -P now"

  boot_wait = "5s"
  boot_command = [
    "c<wait>",
    "linux /casper/vmlinuz --- autoinstall ds=\"nocloud-net;seedfrom=http://{{.HTTPIP}}:{{.HTTPPort}}/\"",
    "<enter><wait>",
    "initrd /casper/initrd",
    "<enter><wait>",
    "boot",
    "<enter>"
  ]
}

build {
  sources = ["sources.vmware-iso.casa_gold_headless"]

  provisioner "shell" {
    timeout = "2h"
    scripts = [
      "../setup_scripts/00_base.sh",
      "../setup_scripts/05_profile.sh",
      "../setup_scripts/10_system_ros2.sh",
      "../setup_scripts/11_system_mavros.sh",
      "../setup_scripts/20_dev_python.sh",
      "../setup_scripts/25_dev_go.sh",
      "../setup_scripts/30_app_network.sh"      
    ]
  }

}
