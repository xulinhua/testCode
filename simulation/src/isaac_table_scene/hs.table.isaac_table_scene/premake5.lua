local ext = get_current_extension_info()
project_ext(ext)

repo_build.prebuild_link {
    { "data", ext.target_dir .. "/data" },
    { "docs", ext.target_dir .. "/docs" },
    { "python/impl", ext.target_dir .. "/hs/table/isaac_table_scene/impl" },
}

repo_build.prebuild_copy {
    { "python/*.py", ext.target_dir .. "/hs/table/isaac_table_scene" },
}
