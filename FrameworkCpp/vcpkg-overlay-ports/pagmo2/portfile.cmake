vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO esa/pagmo2
    REF "v${VERSION}"
    SHA512 9ebe7f63b907607ea5762e56a884be62630efaca3f45d9ba9ad85ca1818d60d09864422bd075c2653aea1a14609fe9ad6520297aee5a00e07fa88df45872cef9
    HEAD_REF master
    PATCHES
        0001-doxygen.patch
        0002-find-tbb.patch
        0003-disable-werror.patch
        0004-support-eigen3-5.patch
)

# Fix: stdext::make_unchecked_array_iterator and stdext::make_checked_array_iterator
# were removed in MSVC 17.5 (VS2022, _MSC_VER >= 1935). Replace the MSVC-specific
# branches with the standard pointer versions which work on all modern compilers.
file(READ "${SOURCE_PATH}/src/problems/translate.cpp" _tc)
string(REPLACE
    "#if defined(_MSC_VER)\n            std::transform(stdext::make_unchecked_array_iterator(xs.data() + i * nx),\n                           stdext::make_unchecked_array_iterator(xs.data() + (i + 1u) * nx),\n                           stdext::make_unchecked_array_iterator(m_translation.data()),\n                           stdext::make_unchecked_array_iterator(xs_deshifted.data() + i * nx), std::minus<double>{});\n#else\n                std::transform(xs.data() + i * nx, xs.data() + (i + 1u) * nx, m_translation.data(),\n                               xs_deshifted.data() + i * nx, std::minus<double>{});\n#endif"
    "std::transform(xs.data() + i * nx, xs.data() + (i + 1u) * nx, m_translation.data(),\n                           xs_deshifted.data() + i * nx, std::minus<double>{});"
    _tc "${_tc}")
file(WRITE "${SOURCE_PATH}/src/problems/translate.cpp" "${_tc}")

file(READ "${SOURCE_PATH}/src/batch_evaluators/thread_bfe.cpp" _bfe)
string(REPLACE
    "#if defined(_MSC_VER)\n                stdext::make_checked_array_iterator(in_ptr, n_dim),\n                stdext::make_checked_array_iterator(in_ptr, n_dim, n_dim), tmp_dv.begin()\n#else\n                in_ptr, in_ptr + n_dim, tmp_dv.begin()\n#endif"
    "in_ptr, in_ptr + n_dim, tmp_dv.begin()"
    _bfe "${_bfe}")
string(REPLACE
    "#if defined(_MSC_VER)\n                fv.begin(), fv.end(), stdext::make_checked_array_iterator(out_ptr, f_dim)\n#else\n                fv.begin(), fv.end(), out_ptr\n#endif"
    "fv.begin(), fv.end(), out_ptr"
    _bfe "${_bfe}")
file(WRITE "${SOURCE_PATH}/src/batch_evaluators/thread_bfe.cpp" "${_bfe}")

vcpkg_check_features(OUT_FEATURE_OPTIONS FEATURE_OPTIONS
    FEATURES
        nlopt PAGMO_WITH_NLOPT
)
string(COMPARE EQUAL "${VCPKG_LIBRARY_LINKAGE}" "static" PAGMO_BUILD_STATIC_LIBRARY)
vcpkg_cmake_configure(
    SOURCE_PATH "${SOURCE_PATH}"
    OPTIONS
        ${FEATURE_OPTIONS}
        -DPAGMO_BUILD_TESTS=OFF
        -DPAGMO_BUILD_BENCHMARKS=OFF
        -DPAGMO_BUILD_TUTORIALS=OFF
        -DPAGMO_WITH_EIGEN3=ON
        -DPAGMO_BUILD_STATIC_LIBRARY=${PAGMO_BUILD_STATIC_LIBRARY}
)

vcpkg_cmake_install()

vcpkg_copy_pdbs()
vcpkg_cmake_config_fixup(CONFIG_PATH "lib/cmake/pagmo")

file(REMOVE_RECURSE "${CURRENT_PACKAGES_DIR}/debug/share")
file(REMOVE_RECURSE "${CURRENT_PACKAGES_DIR}/debug/include")

vcpkg_install_copyright(FILE_LIST "${SOURCE_PATH}/COPYING.lgpl3" "${SOURCE_PATH}/COPYING.gpl3")
file(INSTALL "${CMAKE_CURRENT_LIST_DIR}/usage" DESTINATION "${CURRENT_PACKAGES_DIR}/share/${PORT}")
