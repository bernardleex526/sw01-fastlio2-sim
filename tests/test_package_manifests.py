from conftest import SRC, parse_xml


PACKAGES = {
    "sw01_description": ("urdf", "meshes"),
    "sw01_gazebo": ("worlds", "launch"),
    "sw01_slam": ("config", "launch", "rviz"),
    "sw01_navigation": ("config", "launch", "scripts"),
}


def test_ament_packages_have_matching_names_and_install_rules():
    for name, install_dirs in PACKAGES.items():
        package_dir = SRC / name
        root = parse_xml(package_dir / "package.xml")
        assert root.findtext("name") == name
        assert root.findtext("export/build_type") == "ament_cmake"
        cmake = (package_dir / "CMakeLists.txt").read_text(encoding="utf-8")
        assert "ament_package()" in cmake
        for install_dir in install_dirs:
            assert install_dir in cmake
