fn main() {
    embed_resource::compile_for("src/app.rc", ["s5c-elite"], embed_resource::NONE).manifest_required().unwrap();
}
