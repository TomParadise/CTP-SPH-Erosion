using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;

public class terraingen : MonoBehaviour
{
    //public int width;
    //public int depth;
    private Mesh mesh;
    public Vector3[] vertices;
    public Vector3[] triangles;
    public int width = 25;
    public int depth = 25;
    // Start is called before the first frame update
    void Start()
    {
        StreamReader File = new StreamReader("Assets/Positions/Mesh.txt");
        string text = File.ReadToEnd();
        string[] xyz = text.Split(',');
        vertices = new Vector3[int.Parse(xyz[0])];
        triangles = new Vector3[int.Parse(xyz[1])];
        for (long i = 0; i < vertices.Length; i++)
        {
            vertices[i] = new Vector3(float.Parse(xyz[2 + i * 3]), float.Parse(xyz[2 + i * 3 + 1]), float.Parse(xyz[2 + i * 3 + 2]));
        }
        for(long i = 0; i< triangles.Length; i++)
        {
            triangles[i] = new Vector3(float.Parse(xyz[vertices.Length * 3+ 2 + i * 3]), float.Parse(xyz[vertices.Length * 3 + 2 + i * 3 + 1]), float.Parse(xyz[vertices.Length * 3 + 2 + i * 3 + 2]));
        }

        mesh = new Mesh();
        GetComponent<MeshFilter>().mesh = mesh;
        mesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;

        int[] indexArray = new int[triangles.Length * 3];
        int j = 0;
        for (int i = 0; i < triangles.Length; i++)
        {
            indexArray[j] = (int)triangles[i].x;
            indexArray[j+1] = (int)triangles[i].y;
            indexArray[j+2] = (int)triangles[i].z;

            j += 3;
        }

        mesh.Clear();
        mesh.vertices = vertices;
        mesh.triangles = indexArray;
        mesh.RecalculateNormals();

        Camera cam = Camera.main;
        Vector3 pos = cam.transform.position;
        pos.x = width / 2;
        pos.y = depth / 2;

        Vector3 rot = cam.transform.eulerAngles;

        if (pos.x < 50)
        {
            rot.x = 30;
        }
        else if (pos.x > 100)
        {
            rot.x = 50;
        }

        cam.transform.position = pos;
        cam.transform.eulerAngles = rot;
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
