-- Setup the basic extension variables
local ext = get_current_extension_info()
project_ext (ext)

-- data/ 为扩展内资源副本（raw_data USD/OBJ/贴图）
repo_build.prebuild_link {
    { "data", ext.target_dir.."/data" },
    { "docs", ext.target_dir.."/docs" },
    { "python/impl", ext.target_dir.."/hs/test/isaac_extension_test/impl" },
}

repo_build.prebuild_copy {
    { "python/*.py", ext.target_dir.."/hs/test/isaac_extension_test" },
}
