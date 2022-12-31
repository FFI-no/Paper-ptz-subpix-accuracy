# Copyright (c) 2022 Norwegian Defence Research Establishment (FFI)

from conans import CMake, ConanFile, tools
from conans.tools import load, SystemPackageTool
import re

def get_version():
    try:
        return re.search(r"project\(\S+ VERSION (\d+\.\d+(\.\d+)?).*\)", load("CMakeLists.txt")).group(1).strip()
    except Exception:
        return None

class PTCEEPaperMCAnalysisConan(ConanFile):
    name = "ptcee-paper-mc-analysis"
    version = get_version()
    exports = "README.md"
    settings = "os", "compiler", "build_type", "arch"
    options = {
        "shared": [True, False],
    }
    default_options = {
        "shared":   False,
    }
    generators = "cmake_find_package"
    exports_sources = "*", "!.gitlab-ci.yml"

    def requirements(self):
        self.requires("ptcee/1.0.18")

    def imports(self):
        self.copy("*.so*", src="@libdirs")
