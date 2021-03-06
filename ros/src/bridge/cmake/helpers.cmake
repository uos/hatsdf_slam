function(set_fastsense_root_dir DIR)
    get_filename_component(PARENT_DIR ${CMAKE_SOURCE_DIR} DIRECTORY)
    get_filename_component(FASTSENSE_DIR ${PARENT_DIR} DIRECTORY)
    set(${DIR} ${FASTSENSE_DIR} PARENT_SCOPE)
endfunction()

function(get_absolute VAR DIR)
    get_filename_component(ABS ${DIR} ABSOLUTE)
    set(${VAR} ${ABS} PARENT_SCOPE)
endfunction()