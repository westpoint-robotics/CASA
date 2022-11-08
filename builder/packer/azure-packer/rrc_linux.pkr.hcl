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

  azure_tags = {
    dept = "engineering"
  }

  location = "West US 2"
  vm_size  = "Standard_A1_v2"
}

build {
  sources = ["sources.azure-arm.swarm2_001"]
}
