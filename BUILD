cc_library(
    name = "rs_driver",
    hdrs = glob([
        "**/*.h",
        "**/*.hpp",
    ]),
    includes = ["./src"],
    linkopts = [
        "-lpthread",
        "-lpcap",
        "-lboost_system",
        # Keep these, we might need them later.
        # "-lpcl",
        # "-leigen3",
    ],
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "demo_online",
    srcs = ["demo/demo_online.cpp"],
    visibility = ["//visibility:public"],
    deps = [
        ":rs_driver",
    ],
)

cc_binary(
    name = "demo_pcap",
    srcs = ["demo/demo_pcap.cpp"],
    visibility = ["//visibility:public"],
    deps = [
        ":rs_driver",
    ],
)
