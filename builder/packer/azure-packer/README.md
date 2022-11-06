# Packer for Azure VMs

- [Packer for Azure VMs](#packer-for-azure-vms)
  - [Usage](#usage)
    - [Perquisites](#perquisites)
    - [Azure Login](#azure-login)
    - [Packer Build](#packer-build)
    - [Azure create image](#azure-create-image)
    - [Use the Image to Create a VM](#use-the-image-to-create-a-vm)
  - [References](#references)
  - [References](#references-1)

This folder contains Packer builds for the Azure VM CASA Gold Image.

This builder creates a raw Azure image [vhd file](https://learn.microsoft.com/en-us/azure/virtual-machines/linux/build-image-with-packer) that can be packaged as an Azure Image and used to make a Virtual Machine.

## Usage

### Perquisites

 * [Hashicorp Packer](https://developer.hashicorp.com/packer/tutorials/docker-get-started/get-started-install-cli)

### Azure Login

The Packer ARM builder is set up to use interactive Azure login, so you will need to login from the [Azure command line](https://learn.microsoft.com/en-us/cli/azure/install-azure-cli):

```
az login
```

You should be redirected to a browser where you can login.

### Packer Build 

Make any required changes to the [casa_gold_desktop_0x64_azure.pkr.hcl](./casa_gold_desktop_0x64_azure.pkr.hcl) file

You shouldn't need to change much, other than the deployment region.

Validate and build:

```
packer validate casa_gold_desktop_0x64_azure.pkr.hcl 
packer build casa_gold_desktop_0x64_azure.pkr.hcl 
```

The desktop image will take appx. 1h to build.

At the end, you should see a message like:

```
==> azure-arm.swarm2_001: Cleanup requested, deleting resource group ...
==> azure-arm.swarm2_001: Resource group has been deleted.
Build 'azure-arm.swarm2_001' finished after 47 minutes 43 seconds.

==> Wait completed after 47 minutes 43 seconds

==> Builds finished. The artifacts of successful builds are:
--> azure-arm.swarm2_001: Azure.ResourceManagement.VMImage:

OSType: Linux
StorageAccountLocation: westus2
OSDiskUri: https://swarm002builds.blob.core.windows.net/system/Microsoft.Compute/Images/images/packer-osDisk.da2e8a55-1e18-46e9-b1cd-b0257e90519e.vhd
OSDiskUriReadOnlySas: https://swarm002builds.blob.core.windows.net/system/Microsoft.Compute/Images/images/packer-osDisk.52e02ffc-6d6f-4e75-811f-3f4b3304a5ad.vhd?se=2022-12-06T20%3A21%3A06Z&sig=kEmK3jUm98ELRSl6V8thn5tsEb3y2d9IRWmLqq67cz4%3D&sp=r&sr=b&sv=2018-03-28
TemplateUri: https://swarm002builds.blob.core.windows.net/system/Microsoft.Compute/Images/images/packer-vmTemplate.da2e8a55-1e18-46e9-b1cd-b0257e90519e.json
TemplateUriReadOnlySas: https://swarm002builds.blob.core.windows.net/system/Microsoft.Compute/Images/images/packer-vmTemplate.52e02ffc-6d6f-4e75-811f-3f4b3304a5ad.json?se=2022-12-06T
```

Make note of the `OSDiskUri` as you will need it in next step

### Azure create image

Login to Azure console, navigate to your resource group, and find the `Images` [product](https://portal.azure.com/#view/HubsExtension/BrowseResource/resourceType/Microsoft.Compute%2Fimages)

Select Create Image

Complete the form, entering the `OSDiskUri` as the Storage Blob.  ***Ensure the Region variable is set to the same region as the Blob***

### Use the Image to Create a VM

After the image is created you can use it to launch a VM.  You can change the specs on the VM and also add a data disk.

## References

```
az vm image list -s 22_04-daily-lts  -o table --all 
```

## References

  * [Installing Packer on Ubuntu](https://developer.hashicorp.com/packer/tutorials/docker-get-started/get-started-install-cli)
  * [Authenticating to Azure with Packer](https://developer.hashicorp.com/packer/plugins/builders/azure)
  * [Getting Started with Packer and Azure](https://freshbrewed.science/2021/09/08/packer-linux.html)

