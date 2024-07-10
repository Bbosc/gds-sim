import os
from wafbuild.utils import load

VERSION = "1.0.0"
APPNAME = "gds-sim"

libname = "GdsSim"
srcdir = "src"
blddir = "build"
libdir = "gds_sim"

compiler = "cxx"
optional = [
    "utilslib",  # for managing reading/writing files
    "controllib",  # provides different types of low levels controllers
    "beautifulbullet",  # robotic simulator
    "zmqstream",  # stream across Python/C++
    "optitracklib",  # communicate with the optitrack system
    "frankacontrol",  # control franka robot
]

required_bld = {
    "src/test.cpp": ["UTILSLIB", "BEAUTIFULBULLET", "CONTROLLIB", "ZMQSTREAM"],
    "src/franka_sim.cpp": ["UTILSLIB", "BEAUTIFULBULLET", "CONTROLLIB", "ZMQSTREAM"],
}

def options(opt):
    # Add build shared library options
    opt.add_option("--shared",
                   action="store_true",
                   help="build shared library")

    # Add build static library options
    opt.add_option("--static",
                   action="store_true",
                   help="build static library")

    # Load library options
    load(opt, compiler, required=None, optional=optional)

def configure(cfg):
    # Load library configurations
    load(cfg, compiler, required=None, optional=optional)

def build(bld):
    sources = []
    for root, _, filenames in os.walk('src'):
        sources += [os.path.join(root, filename) for filename in filenames if filename.endswith(('.cpp', '.cc'))]

    for example in sources:
        if example in required_bld:
            if set(required_bld[example]).issubset(bld.env["libs"]):
                bld.program(
                    features="cxx",
                    source=example,
                    uselib=bld.env["libs"],
                    target=example[:-len(".cpp")],
                )
        else:
            bld.program(
                features="cxx",
                source=example,
                uselib=bld.env["libs"],
                target=example[:-len(".cpp")],
            )

