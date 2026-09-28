#!/usr/bin/env python
# 构建 XPBD 布料 C++ 扩展 (GDExtension)
#   ../.buildenv/bin/scons -j4 platform=macos arch=x86_64 target=template_debug
# 依赖同级 ../godot-cpp (api_version=4.7)

import os

env = SConscript("../godot-cpp/SConstruct", {"api_version": "4.7"})

env.Append(CPPPATH=["src/cloth/"])
sources = Glob("src/cloth/*.cpp")

bin_dir = "addons/cloth_native/bin"
if env["platform"] == "macos":
    library = env.SharedLibrary(
        "{}/libcloth_native.{}.{}.framework/libcloth_native.{}.{}".format(
            bin_dir, env["platform"], env["target"], env["platform"], env["target"]
        ),
        source=sources,
    )
else:
    suffix = env["suffix"].replace(".dev", "").replace(".universal", "")
    library = env.SharedLibrary(
        "{}/libcloth_native{}{}".format(bin_dir, env["suffix"], env["SHLIBSUFFIX"]),
        source=sources,
    )

Default(library)
