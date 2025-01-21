# In this file we define some functions that are used in the root CMakeLists.txt files

################### Function to add a submodule ###################
function(add_submodule name)
    find_package(Git QUIET)
    if(GIT_FOUND AND EXISTS "${PROJECT_SOURCE_DIR}/.git")
        # Update submodules as needed
        option(GIT_SUBMODULE "Check submodules during build" ON)
        if(GIT_SUBMODULE)
            message(STATUS "Updating submodule ${name}...")
            execute_process(COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive submodules/${name}
                            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                            RESULT_VARIABLE GIT_SUBMOD_RESULT)
            if(NOT GIT_SUBMOD_RESULT EQUAL "0")
                message(FATAL_ERROR "git submodule update --init --recursive ${name} failed with result: ${GIT_SUBMOD_RESULT}. Please check out submodules manually.")
            endif()
        endif()
    else()
        message(WARNING "Git not found or not a git repository. Skipping submodule update for ${name}.")
    endif()

    if(NOT EXISTS "${PROJECT_SOURCE_DIR}/submodules/${name}/CMakeLists.txt")
        message(FATAL_ERROR "The submodule ${name} was not downloaded! GIT_SUBMODULE was turned off or failed. Please update submodules and try again.")
    endif()

    add_subdirectory(submodules/${name})
endfunction()