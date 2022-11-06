source "azure-arm" "swarm2_001" {
  use_interactive_auth = "true"
  resource_group_name  = "SomeRG"
  storage_account      = "swarm002builds"
  subscription_id      = "69b4a217-b85d-4344-95fc-cf86efa2768c"
  tenant_id            = "ea151704-9fd0-454d-887f-6e418cd14cf3"

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
