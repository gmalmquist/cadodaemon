mod mesh;

fn main() {
    println!("Hello, world!");

    let obj = obj::Obj::load(std::path::Path::new("./test-meshes/cube.obj"))
        .expect("Failed to load test input file ./test-meshes/cube.obj");
    let mesh: mesh::Mesh = obj.into();
    let obj: obj::Obj = mesh.into();

    obj.save(std::path::Path::new("./test-meshes/output.obj"))
        .expect("Failed to save output obj.");

    println!("Done.");
}
