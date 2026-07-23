import os

Import("env")

# The only purpose of this extra script is to set the `NRF_CRYPTOCELL` macro for the
# nrf52840 series because the original `NRF_CRYPTOCELL` macro is set in the framework
# sources and not visible when PlatformIO LDF resolves dependencies.
if env.BoardConfig().get("build.mcu", "").startswith("nrf5284"):
    env.Append(
        CPPDEFINES=[("NRF_CRYPTOCELL", "((NRF_CRYPTOCELL_Type*) NRF_CRYPTOCELL_BASE)")],

    )
