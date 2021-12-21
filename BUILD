load("@rules_foreign_cc//foreign_cc:defs.bzl", "cmake")

filegroup(
    name = "all_srcs",
    srcs = glob(["**"]),
    visibility = ["//visibility:public"],
)

cmake(
    name = "rs_driver",
    cache_entries = {
        "CMAKE_C_FLAGS": "-fPIC",
    },
    install = False,
    lib_source = ":all_srcs",
    out_headers_only = True,
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "demo",
    srcs = ["demo/demo_online.cpp"],
    visibility = ["//visibility:public"],
    deps = [
        ":rs_driver",
    ],
)
