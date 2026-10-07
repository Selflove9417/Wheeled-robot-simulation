# Frozen source manifest

Source package: `ignition-gazebo6 6.18.0-1~jammy` (upstream Gazebo Sim 6.18.0).
The Debian source package includes no `debian/patches` series; its packaging
archive does not patch `src/systems/physics/Physics.cc`.

| File | SHA-256 |
|---|---|
| `upstream/ignition-gazebo6_6.18.0-1~jammy.dsc` | `15ccf27a74f24dc4204215f88f927ed2d2b2bb5ebaba89b2d7f695e3b07d3ee1` |
| `upstream/ignition-gazebo6_6.18.0.orig.tar.bz2` | `9d3bbadc1a9c780b179890f4ac5978195d98a7a12f4cc23614d895276774fc99` |
| `upstream/ignition-gazebo6_6.18.0-1~jammy.debian.tar.xz` | `9a8b169b496a488c732ec839ce5dc451350fc2be7d4d9da56c68399d04ba3619` |
| Modified `upstream/src/systems/physics/Physics.cc` | `7d1ff9eb5d3327569d59d137017620837b78b233b3aefac85352bc49c3aa5a94` |
| `patches/raw_native_readonly.patch` | `e55341ba338c679c3a6f58ad9e68666a6ac1284fa091083559c1678477cb4804` |
| Built private plugin | `3e7140cffb1c8ca7f229327a10b91450689904ca690acd4c9f5e4ef66f7fefb9` |

`Physics.hh` and the two private local helper headers are copied unchanged from
the same upstream source archive. The target was configured and compiled
against the installed Gazebo Sim 6.18 / Ignition Physics 5.4 dependencies.
