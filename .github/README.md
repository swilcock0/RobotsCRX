<div align="center">
    
# ![Robots](../docs/Images/robots-icon.svg)<br/>RobotsCRX
**A fork of [Robots](https://github.com/visose/Robots) with FANUC CRX support and Stream Motion streaming**

**Create and simulate ABB, KUKA, UR, Staubli, Franka Emika, Doosan, Fanuc, Igus, and Jaka robot programs in Rhino 8, Grasshopper, and .NET**

[![License](https://img.shields.io/github/license/swilcock0/RobotsCRX?style=flat-square)](../LICENSE)
[![Release](https://img.shields.io/github/v/release/swilcock0/RobotsCRX?include_prereleases&style=flat-square)](https://github.com/swilcock0/RobotsCRX/releases)
[![Upstream](https://img.shields.io/badge/upstream-visose%2FRobots-gray?style=flat-square)](https://github.com/visose/Robots)
[![Sponsor](https://img.shields.io/badge/sponsor-gray?style=flat-square&logo=GitHub-Sponsors)](https://github.com/sponsors/visose)

**[RobotsCRX](#robotscrx) •
[About](#about) •
[Install](#install) •
[Docs](https://github.com/visose/Robots/wiki) •
[Support](#support) •
[Supporting Robots](#supporting-robots)**

</div>

## RobotsCRX

RobotsCRX is a fork of **[Robots](https://github.com/visose/Robots)** by **Vicente Soler ([visose](https://github.com/visose))** and the Robots authors. All credit for the plugin goes to them. Thank you for making Robots open source. If Robots is useful to you, please consider [sponsoring the original project](https://github.com/sponsors/visose). For issues with Robots itself, use the [upstream repository](https://github.com/visose/Robots).

The fork adds:

- **FANUC CRX robots**: the CRX-3iA, CRX-5iA, CRX-10iA, CRX-10iA/L, CRX-10iA/L Paint, CRX-20iA/L and CRX-30iA ship with the plugin, converted from FANUC’s [fanuc_description](https://github.com/FANUC-CORPORATION/fanuc_description) (Apache-2.0) by `tools/Robots.FanucCrx`, and solved with the analytical non-spherical wrist solver.
- **Stream Motion**: the `Remote connection` component streams programs to FANUC controllers with Stream Motion (J519), starting the controller’s `STREAM_MOTN` program through RMI. Upload checks the program against the controller’s own joint limits, Play moves slowly to the program start if needed and checks the robot model against the controller first, and Pause stops smoothly.
- **Read Joints**: a component that reads the robot’s current joints from the controller, once or continuously.

> Streaming moves a real robot. Test with low speeds and override, with the teach pendant at hand.

> Streaming needs the controller set up as for FANUC’s ROS 2 driver. See its [system requirements](https://fanuc-corporation.github.io/fanuc_driver_doc/main/docs/environment/system_requirements.html) for the software options and version, and its [robot controller setup](https://fanuc-corporation.github.io/fanuc_driver_doc/main/docs/quick_start/quick_start.html#robot-controller-setup) for the checks before connecting.

> Like the other Fanuc robots in Robots, a CRX’s origin (the plane given to `Load robot system`) is FANUC’s world frame at the height of the J2 axis, not the mounting surface: 161 mm above it for the CRX-3iA, 185 mm for the CRX-5iA, 245 mm for the CRX-10iA, CRX-10iA/L and CRX-20iA/L, 370 mm for the CRX-30iA and 465 mm for the CRX-10iA/L Paint. Cartesian targets then match the controller’s world coordinates.

### Install RobotsCRX

1. Uninstall `Robots` in the Rhino 8 `_PackageManager` if it is installed; RobotsCRX replaces it.
1. Download the `.yak` file from the [latest release](https://github.com/swilcock0/RobotsCRX/releases) and drag it onto the Rhino window, or run `yak install <file>.yak`.
1. Restart Rhino and open Grasshopper. The CRX robots appear in the `Load robot system` list.
1. To stream to a robot, set up the controller following FANUC’s ROS 2 driver documentation: the [system requirements](https://fanuc-corporation.github.io/fanuc_driver_doc/main/docs/environment/system_requirements.html) (Stream Motion J519 and Remote Motion R912, or S636, and the controller software version) and the [robot controller setup](https://fanuc-corporation.github.io/fanuc_driver_doc/main/docs/quick_start/quick_start.html#robot-controller-setup) (network and controller status). RobotsCRX starts the controller’s `STREAM_MOTN` program itself, as FANUC’s driver does.

To build the package yourself, run `build/pack-crx.ps1` (requires the .NET SDK in `global.json` and Rhino 8).

RobotsCRX has its own version numbers. Each release notes the Robots version it is based on; versions below 1.0 have not yet been validated on a real robot.

### Acknowledgements

- **[Robots](https://github.com/visose/Robots)** by Vicente Soler ([visose](https://github.com/visose)) and the Robots authors, including the analytical non-spherical wrist solver (`NonSphericalWristKinematics`) that RobotsCRX uses for the CRX.
- **FANUC** for [fanuc_description](https://github.com/FANUC-CORPORATION/fanuc_description), the source of the CRX meshes and dimensions, and the [FANUC ROS 2 Driver](https://github.com/FANUC-CORPORATION/fanuc_driver), the reference for the Stream Motion and RMI protocols (both Apache-2.0).
- **Abbes and Poisson**, [Geometric Approach for Inverse Kinematics of the FANUC CRX Collaborative Robot](https://www.mdpi.com/2218-6581/13/6/91), *Robotics* 13(6), 91, 2024, and **Daniel Cranston’s** [crx_kinematics](https://github.com/danielcranston/crx_kinematics), its implementation, against which the CRX geometry was cross-checked. RobotsCRX uses Robots’ own solver rather than this method.

## About

**Robots** is a plugin for **[Rhino 8](https://www.rhino3d.com/)** and the **Grasshopper** visual programming interface, with a .NET API for use in custom applications and automation. It lets users create, check, simulate, and save manufacturer-specific robot programs.

Supported manufacturers include **ABB**, **KUKA**, **UR**, **Staubli**, **Franka Emika**, **Doosan**, **Fanuc**, **Igus**, and **Jaka**. Robots can load robot systems, tools, frames, and meshes from XML/3DM libraries, use custom post-processors, and generate manufacturer-specific robot code.

## Install

> If upgrading from an old version check [here](https://github.com/visose/Robots/wiki/home#Upgrading-from-an-older-version).

### Grasshopper plugin

1. Install **RobotsCRX** in **Rhino 8** for **Windows** or **macOS** as described in [Install RobotsCRX](#install-robotscrx).
1. Restart **Rhino** and open **Grasshopper**. There should be a new tab in **Grasshopper** named `Robots`.
1. Install a robot library by clicking on the `Libraries` button of a `Load robot system` component.
   > The robots from the library should appear in a **value list** connected to a `Load robot system` component.
1. Read the [docs](https://github.com/visose/Robots/wiki) for more info.
1. Check the [samples](../samples/).
   > When opening a sample file, a dialog box might pop up with an assembly not found message. You can close this without fixing the path, it will automatically get fixed after the sample file is loaded.

### .NET packages

- [`Robots`](https://www.nuget.org/packages/Robots) is the core .NET 8 package built on `Rhino3dm` for use outside Rhino.
- [`Robots.Rhino`](https://www.nuget.org/packages/Robots.Rhino) provides compile-time references for developing Rhino and Grasshopper plug-ins.


## Support

### RobotsCRX

For the CRX robots, Stream Motion streaming or Read Joints, open an [issue](https://github.com/swilcock0/RobotsCRX/issues) in this repository.

### 🆓 [Robots community](https://github.com/visose/Robots/discussions)

For questions about Robots itself, use the upstream discussions forum.
### 💼 [Commercial](https://visose.com/robots)

Commercial support for Robots is offered by its creators. You can find more information [here](https://visose.com/robots).

## Supporting Robots

Robots, which RobotsCRX builds on, is made possible by its sponsors and commercial partners, listed in the [Robots README](https://github.com/visose/Robots#sponsors). RobotsCRX has no sponsors of its own. If you find it useful, please support the original project by purchasing [commercial support](https://visose.com/robots), or by donating via [GitHub Sponsors](https://github.com/sponsors/visose) or [PayPal](https://visose.com/paypal).
