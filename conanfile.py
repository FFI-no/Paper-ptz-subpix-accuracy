# Copyright (c) 2025 Norwegian Defence Research Establishment (FFI)

from conan import ConanFile
from conan.tools.cmake import cmake_layout, CMake, CMakeDeps, CMakeToolchain

class ptceePaperMCAnalysisRecipe(ConanFile):
    name = "ptcee-paper-mc-analysis"
    package_type = "application"

    settings = "os", "compiler", "build_type", "arch"
    exports_sources = "CMakeLists.txt", "cmake/*", "include/*", "src/*"

    options = {
        "shared": [True, False],
    }

    default_options = {
        "shared": False,
        "opencv/*:ml": False,
        "opencv/*:dnn": False,
        "opencv/*:gapi": False,
        "opencv/*:video": False,
        "opencv/*:videoio": False,
        "opencv/*:highgui": False,
        "opencv/*:imgcodecs": False,
        "opencv/*:objdetect": False,
    }

    def set_version(self):
        from conan.tools.files import load
        import os
        import re
        self.version = re.search(r"project\([\S\s]+ VERSION (\d+(\.\d+){1,3})",
                                 load(self, os.path.join(self.recipe_folder, "CMakeLists.txt"))).group(1).strip()

    def build_requirements(self):
        self.tool_requires("cmake/[>=3.23.5]")

    def requirements(self):
        self.requires("opencv/4.10.0")
        self.requires("ptcee/1.0.0")

    def generate(self):
        deps = CMakeDeps(self)
        deps.generate()
        tc = CMakeToolchain(self)
        tc.generate()

    def layout(self):
        cmake_layout(self)

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        cmake = CMake(self)
        cmake.install()

    def package_info(self):
        self.cpp_info.libs = [self.name]
