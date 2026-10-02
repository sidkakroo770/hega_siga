// Diagnostic: inspect what Gazebo's own importer produces from the GLB.
#include <gz/common/MeshManager.hh>
#include <gz/common/Mesh.hh>
#include <gz/common/SubMesh.hh>
#include <gz/common/Material.hh>
#include <iostream>
int main(int argc, char **argv) {
  if (argc != 2) return 2;
  const auto *mesh = gz::common::MeshManager::Instance()->Load(argv[1]);
  if (!mesh) return 1;
  for (unsigned i = 0; i < mesh->MaterialCount(); ++i)
    std::cout << *mesh->MaterialByIndex(i) << '\n';
  for (unsigned i = 0; i < mesh->SubMeshCount(); ++i) {
    auto sub = mesh->SubMeshByIndex(i).lock();
    double area = 0;
    for (unsigned j = 0; j + 2 < sub->IndexCount(); j += 3) {
      auto a = sub->Vertex(sub->Index(j));
      auto b = sub->Vertex(sub->Index(j + 1));
      auto c = sub->Vertex(sub->Index(j + 2));
      area += (b-a).Cross(c-a).Length()/2;
    }
    std::cout << sub->Name() << " vertices=" << sub->VertexCount()
              << " indices=" << sub->IndexCount() << " area=" << area
              << " min=" << sub->Min() << " max=" << sub->Max() << '\n';
  }
}
