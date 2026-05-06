-- Axiom-Physics build script
-- Generate project files with: vendor\bin\premake5.exe vs2022   (or run Setup.bat)

workspace "Axiom-Physics"
    architecture "x86_64"
    startproject "Axiom-Physics"

    configurations
    {
        "Debug",
        "Release",
        "Dist"
    }

    flags
    {
        "MultiProcessorCompile"
    }

outputdir = "%{cfg.buildcfg}-%{cfg.system}-%{cfg.architecture}"

project "Axiom-Physics"
    location "."
    kind "StaticLib"
    language "C++"
    cppdialect "C++latest"
    cdialect "C17"
    staticruntime "off"

    targetdir ("bin/" .. outputdir .. "/%{prj.name}")
    objdir    ("bin-int/" .. outputdir .. "/%{prj.name}")

    files
    {
        "include/**.hpp",
        "include/**.h",
        "src/**.cpp",
        "src/**.hpp",
        "src/**.h"
    }

    includedirs
    {
        "include",
        "external/include"
    }

    filter "system:windows"
        systemversion "latest"
        defines { "_CRT_SECURE_NO_WARNINGS" }

    filter "configurations:Debug"
        defines { "AXIOM_PHYS_DEBUG", "_DEBUG" }
        runtime "Debug"
        symbols "on"
        optimize "off"

    filter "configurations:Release"
        defines { "AXIOM_PHYS_RELEASE", "NDEBUG" }
        runtime "Release"
        symbols "on"
        optimize "on"

    filter "configurations:Dist"
        defines { "AXIOM_PHYS_DIST", "NDEBUG" }
        runtime "Release"
        symbols "off"
        optimize "full"
