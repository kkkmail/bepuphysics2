using System;
using System.Collections.Generic;
using System.Numerics;
using BepuPhysics.Collidables;
using BepuUtilities.Memory;
using BepuUtilities;

namespace Demos;

// public static class PlanetMeshCreator
// {
//     public static Mesh CreatePlanetMesh(float radius, int subdivisionSteps, Vector3 scaling, BufferPool pool, IThreadDispatcher dispatcher = null)
//     {
//         // Create initial icosahedron
//         var vertices = new List<Vector3>();
//         var triangles = new List<(int, int, int)>();
//
//         CreateIcosahedron(vertices, triangles);
//
//         // Normalize vertices to unit sphere and scale by radius
//         for (int i = 0; i < vertices.Count; i++)
//         {
//             vertices[i] = Vector3.Normalize(vertices[i]) * radius;
//         }
//
//         // Subdivide the mesh
//         for (int step = 0; step < subdivisionSteps; step++)
//         {
//             SubdivideMesh(vertices, triangles, radius);
//         }
//
//         // Convert to BepuPhysics format
//         pool.Take<Triangle>(triangles.Count, out var bepuTriangles);
//
//         for (int i = 0; i < triangles.Count; i++)
//         {
//             var (a, b, c) = triangles[i];
//             ref var triangle = ref bepuTriangles[i];
//             triangle.A = vertices[a];
//             triangle.B = vertices[b];
//             triangle.C = vertices[c];
//         }
//
//         return new Mesh(bepuTriangles, scaling, pool, dispatcher);
//     }
//
//     private static void CreateIcosahedron(List<Vector3> vertices, List<(int, int, int)> triangles)
//     {
//         // Golden ratio
//         float phi = (1.0f + MathF.Sqrt(5.0f)) / 2.0f;
//
//         // Create 12 vertices of icosahedron
//         vertices.AddRange(new Vector3[]
//         {
//             new Vector3(-1,  phi,  0), new Vector3( 1,  phi,  0), new Vector3(-1, -phi,  0), new Vector3( 1, -phi,  0),
//             new Vector3( 0, -1,  phi), new Vector3( 0,  1,  phi), new Vector3( 0, -1, -phi), new Vector3( 0,  1, -phi),
//             new Vector3( phi,  0, -1), new Vector3( phi,  0,  1), new Vector3(-phi,  0, -1), new Vector3(-phi,  0,  1)
//         });
//
//         // Create 20 triangular faces of icosahedron
//         triangles.AddRange(new (int, int, int)[]
//         {
//             // 5 faces around point 0
//             (0, 11, 5), (0, 5, 1), (0, 1, 7), (0, 7, 10), (0, 10, 11),
//
//             // 5 adjacent faces
//             (1, 5, 9), (5, 11, 4), (11, 10, 2), (10, 7, 6), (7, 1, 8),
//
//             // 5 faces around point 3
//             (3, 9, 4), (3, 4, 2), (3, 2, 6), (3, 6, 8), (3, 8, 9),
//
//             // 5 adjacent faces
//             (4, 9, 5), (2, 4, 11), (6, 2, 10), (8, 6, 7), (9, 8, 1)
//         });
//     }
//
//     private static void SubdivideMesh(List<Vector3> vertices, List<(int, int, int)> triangles, float radius)
//     {
//         var newTriangles = new List<(int, int, int)>();
//         var midpointsCache = new Dictionary<(int, int), int>();
//
//         foreach (var (a, b, c) in triangles)
//         {
//             // Get midpoints of each edge
//             int ab = GetMidpoint(a, b, vertices, midpointsCache, radius);
//             int bc = GetMidpoint(b, c, vertices, midpointsCache, radius);
//             int ca = GetMidpoint(c, a, vertices, midpointsCache, radius);
//
//             // Create 4 new triangles from 1 original triangle
//             newTriangles.Add((a, ab, ca));
//             newTriangles.Add((b, bc, ab));
//             newTriangles.Add((c, ca, bc));
//             newTriangles.Add((ab, bc, ca));
//         }
//
//         triangles.Clear();
//         triangles.AddRange(newTriangles);
//     }
//
//     private static int GetMidpoint(int index1, int index2, List<Vector3> vertices, Dictionary<(int, int), int> cache, float radius)
//     {
//         // Ensure consistent ordering for cache key
//         var key = index1 < index2 ? (index1, index2) : (index2, index1);
//
//         if (cache.TryGetValue(key, out int cachedIndex))
//         {
//             return cachedIndex;
//         }
//
//         // Calculate midpoint and project to sphere surface
//         Vector3 midpoint = (vertices[index1] + vertices[index2]) * 0.5f;
//         midpoint = Vector3.Normalize(midpoint) * radius;
//
//         int newIndex = vertices.Count;
//         vertices.Add(midpoint);
//         cache[key] = newIndex;
//
//         return newIndex;
//     }
// }

public static class PlanetMeshCreator
{
    public static Mesh CreatePlanetMesh(float radius, int subdivisionSteps, Vector3 scaling, BufferPool pool, IThreadDispatcher dispatcher = null)
    {
        // Create initial icosahedron
        var vertices = new List<Vector3>();
        var triangles = new List<(int, int, int)>();

        CreateIcosahedron(vertices, triangles);

        // Normalize vertices to unit sphere and scale by radius
        for (int i = 0; i < vertices.Count; i++)
        {
            vertices[i] = Vector3.Normalize(vertices[i]) * radius;
        }

        // Subdivide the mesh
        for (int step = 0; step < subdivisionSteps; step++)
        {
            SubdivideMesh(vertices, triangles, radius);
        }

        // Convert to BepuPhysics format
        pool.Take<Triangle>(triangles.Count, out var bepuTriangles);

        for (int i = 0; i < triangles.Count; i++)
        {
            var (a, b, c) = triangles[i];
            ref var triangle = ref bepuTriangles[i];
            triangle.A = vertices[a];
            triangle.B = vertices[b];
            triangle.C = vertices[c];
        }

        return new Mesh(bepuTriangles, scaling, pool, dispatcher);
    }

    private static void CreateIcosahedron(List<Vector3> vertices, List<(int, int, int)> triangles)
    {
        // Golden ratio
        float phi = (1.0f + MathF.Sqrt(5.0f)) / 2.0f;

        // Create 12 vertices of icosahedron
        vertices.AddRange(new Vector3[]
        {
            new Vector3(-1,  phi,  0), new Vector3( 1,  phi,  0), new Vector3(-1, -phi,  0), new Vector3( 1, -phi,  0),
            new Vector3( 0, -1,  phi), new Vector3( 0,  1,  phi), new Vector3( 0, -1, -phi), new Vector3( 0,  1, -phi),
            new Vector3( phi,  0, -1), new Vector3( phi,  0,  1), new Vector3(-phi,  0, -1), new Vector3(-phi,  0,  1)
        });

        // Create 20 triangular faces of icosahedron
        triangles.AddRange(new (int, int, int)[]
        {
            // 5 faces around point 0
            (0, 11, 5), (0, 5, 1), (0, 1, 7), (0, 7, 10), (0, 10, 11),

            // 5 adjacent faces
            (1, 5, 9), (5, 11, 4), (11, 10, 2), (10, 7, 6), (7, 1, 8),

            // 5 faces around point 3
            (3, 9, 4), (3, 4, 2), (3, 2, 6), (3, 6, 8), (3, 8, 9),

            // 5 adjacent faces
            (4, 9, 5), (2, 4, 11), (6, 2, 10), (8, 6, 7), (9, 8, 1)
        });
    }

    private static void SubdivideMesh(List<Vector3> vertices, List<(int, int, int)> triangles, float radius)
    {
        var newTriangles = new List<(int, int, int)>();
        var midpointsCache = new Dictionary<(int, int), int>();

        foreach (var (a, b, c) in triangles)
        {
            // Get midpoints of each edge
            int ab = GetMidpoint(a, b, vertices, midpointsCache, radius);
            int bc = GetMidpoint(b, c, vertices, midpointsCache, radius);
            int ca = GetMidpoint(c, a, vertices, midpointsCache, radius);

            // Create 4 new triangles from 1 original triangle
            newTriangles.Add((a, ab, ca));
            newTriangles.Add((b, bc, ab));
            newTriangles.Add((c, ca, bc));
            newTriangles.Add((ab, bc, ca));
        }

        triangles.Clear();
        triangles.AddRange(newTriangles);
    }

    private static int GetMidpoint(int index1, int index2, List<Vector3> vertices, Dictionary<(int, int), int> cache, float radius)
    {
        // Ensure consistent ordering for cache key
        var key = index1 < index2 ? (index1, index2) : (index2, index1);

        if (cache.TryGetValue(key, out int cachedIndex))
        {
            return cachedIndex;
        }

        // Calculate midpoint and project to sphere surface using float precision
        Vector3 point1 = vertices[index1];
        Vector3 point2 = vertices[index2];
        Vector3 midpoint = (point1 + point2) * 0.5f;
        midpoint = Vector3.Normalize(midpoint) * radius;

        int newIndex = vertices.Count;
        vertices.Add(midpoint);
        cache[key] = newIndex;

        return newIndex;
    }
}
