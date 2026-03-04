file(GLOB_RECURSE COMPONENT_SRCS "kinematic/*.c" "socket/*.c" "motion_control/*.c" "bno055/*.c" "position/*.c")

idf_component_register(
    SRCS ${COMPONENT_SRCS} "main.c"
    INCLUDE_DIRS "../inc" "../inc/bno055"
)