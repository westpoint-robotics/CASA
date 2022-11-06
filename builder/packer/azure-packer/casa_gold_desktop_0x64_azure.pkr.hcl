

/////////////// VARIABLES //////////////////

variable "password" {
  type = string
  default = "AVeryHardToCrackPassword"
}

variable "local_iso_folder" {
  type    = string
  default = "/tmp/swarmiso"
}

variable "output_directory" {
  type    = string
  default = "/tmp/swarm_build_output"
}


source "azure-arm" "swarm2_001" {
  use_interactive_auth = "true"
  resource_group_name  = "EECS-RRC"
  storage_account      = "swarm002builds"
  subscription_id      = "139d1fac-0e90-4718-84e5-88b774195742"
  tenant_id            = "99ff8811-3517-40a9-bf10-45ea0a321f0b"

  capture_container_name = "images"
  capture_name_prefix    = "packer"

  os_type         = "Linux"
  image_publisher = "Canonical"
  image_offer     = "0001-com-ubuntu-server-jammy-daily"
  image_sku       = "22_04-daily-lts"

  ssh_username = "analyst"

  azure_tags = {
    dept = "RRC"
    owner = "Steve Henderson"
  }

  location = "West US 2"
  vm_size  = "Standard_A1_v2"
}

build {
  sources = ["sources.azure-arm.swarm2_001"]
  provisioner "shell" {
    inline = [
      "sudo apt update",
      "sudo apt-get update"      
    ]
  }
  provisioner "shell" {
    timeout = "2h"
    scripts = [
      "../setup_scripts/00_base_azure.sh",
      "../setup_scripts/05_profile.sh",
      "../setup_scripts/10_system_ros2.sh",
      "../setup_scripts/11_system_mavros.sh",
      "../setup_scripts/20_dev_python.sh",
      "../setup_scripts/25_dev_go.sh",
      "../setup_scripts/30_app_network.sh",      
      "../setup_scripts/100_ubuntu_desktop.sh",            
      "../setup_scripts/101_vmware_tools_desktop.sh",            
      "../setup_scripts/135_app_vscode.sh"
    ]
  }
}
