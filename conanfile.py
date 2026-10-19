from conan import ConanFile
from conan.tools.cmake import CMakeDeps, cmake_layout


class RpApplicationConan(ConanFile):
    name = "rp-application"
    settings = "os", "arch", "compiler", "build_type"

    def requirements(self):
        self.requires("rp-midware/[>=0.1.0 <0.2.0]")

    def generate(self):
        CMakeDeps(self).generate()

    def layout(self):
        cmake_layout(self)
