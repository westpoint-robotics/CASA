# CASA Packer Image Builder for VMWare

- [CASA Packer Image Builder for VMWare](#casa-packer-image-builder-for-vmware)
  - [Pre-requisites](#pre-requisites)
  - [Usage](#usage)
  - [Building Images](#building-images)
  - [Wrapping up](#wrapping-up)
  - [Advanced Config](#advanced-config)
    - [Setting a new password (Optional)](#setting-a-new-password-optional)
  - [TODO](#todo)
  - [Troubleshooting](#troubleshooting)
  - [References](#references)

This folder contains Packer build instructions for creating CASA VMWare machine images.


## Pre-requisites

  * [Hashicorp Packer](https://developer.hashicorp.com/packer/tutorials/docker-get-started/get-started-install-cli)
  * [VMWare Workstation](https://www.vmware.com/products/workstation-pro.html)

## Usage

Ensure the following ENV are set

```
export PKR_VAR_password=SomePassword
export PKR_VAR_iso_target_path=/home/users/Downloads
export PKR_VAR_output_directory=/some/folder/for/results
```

Notes:

1) The `PKR_VAR_output_directory` is a destructive write, so ensure you select a new, empty folder!

2) If you want to use the default password, contact [@stevehenderson](https://github.com/stevehenderson) and place it in the `PKR_VAR_password`.  Else see Advanced Config below


## Building Images

There are a number of examples for various builds:


   * `swarm2_gold_headless_0x64_vmware.pkr.hcl` : A headless image suitable for deployment on a 0x64/AMD64 hardware device or in a simulated virtual device (cloud or local VM)
  
   * `swarm2_gold_desktop_0x64_vmware.pkr.hcl` : A desktop version suitable for developing an testing with CASA

There is a configuration section in each file that you may want to tweak:


```
  // Virtual Hardware Specs
  memory       = 8172
  cpus         = 1
  cores        = 4
  disk_size    = 32000
  sound        = true
  disk_type_id = 0

```

Select (and optionally edit) the `pkr.hcl` file.  Then run the following from the `vmware-packer` folder:

```
packer init .
packer format .
packer validate .
```

Then

```
packer build -f <hcl file>
```

i.e.

```
packer build -f swarm2_gold_headless_0x64_vmware.pkr.hcl
```

## Wrapping up

The output folder set in your ENV's should contain the final VMWare image and `vmx` files.  

**Ensure you move it to a safe place as it will be blown away if you run packer again**

## Advanced Config

### Setting a new password (Optional)

The `PKR_VAR_password` must reflect the password hash in the `./http/user-data` file:


```
.
.
password: $6$oRUQ2cjmKIEo$XiRnTkX1eHMKJPS9jnyR0O3LcxTK5GRyCppINaHvYWTPm7vJw5hlbc3nNbzVYvu/Db8vAczKlkE2ijoDaM3pr0
.
.
```

A new one can be generated as follows:

```
echo 'MyP@ssw0rd-22!' | mkpasswd -m sha-512 --stdi
```

## TODO

  * add VMWare tools
  
  * add ros rqt

    ```
    sudo apt install ~nros-foxy-rqt*
    ```

## Troubleshooting

0.  Ensure your ENV are set
1.  Try running the packer build again

## References

  * [ubuntu-22-04-packer-fusion-workstation from burkeazbill](https://github.com/burkeazbill/ubuntu-22-04-packer-fusion-workstation)
  * [VMWare User Group](https://www.vmug.com/membership/membership-benefits/)