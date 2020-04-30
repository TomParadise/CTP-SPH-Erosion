using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;

public class terraingen : MonoBehaviour
{
    [SerializeField] private bool simulateParticles;
    //public int width;
    //public int depth;
    private Mesh mesh;
    private Vector3[] vertices;
    private Vector3[] triangles;
    public int width = 100;
    public int depth = 100;
    
    private StreamReader File;
    private string text;
    public int frame = 0;
    private float timer = 0.0f;
    [SerializeField] private int frameCount;
    float fps = 1 / 15;

    public bool updateMesh = false;

    // Start is called before the first frame update
    void Start()
    {
        GetComponent<SPHSimulation>().particleCount = width * depth * 2;
        GetComponent<SPHSimulation>().frameCount = frameCount;
        if (simulateParticles)
        {
            GetComponent<SPHSimulation>().enabled = true;
        }
        resetMesh();

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
        if (frame < frameCount)
        {
            timer += Time.deltaTime;
            if (timer >= fps)
            {
                if (frame > 1)
                {
                    File = new StreamReader("Assets/Positions/Mesh" + frame.ToString() + ".txt");
                    text = File.ReadToEnd();
                    string[] xyz = text.Split(',');
                    for (long i = 0; i < vertices.Length; i++)
                    {
                        vertices[i] = new Vector3(float.Parse(xyz[i * 3]), float.Parse(xyz[i * 3 + 1]), float.Parse(xyz[i * 3 + 2]));
                    }
                }
                frame++;
                if(frame == frameCount)
                {
                    Debug.Log("finished");
                }

                timer = 0.0f;
            }
            mesh.vertices = vertices;
            mesh.RecalculateNormals();
        }
        //else
        //{
        //    frame = 1;
        //}
        //if(updateMesh)
        //{
        //    if(frame != frameCount)
        //    {
        //        frame = frameCount;
        //    }
        //}
        //else
        //{
        //    if(frame != 0)
        //    {
        //        frame = 0;
        //        resetMesh();
        //    }
        //}
    }

    private void resetMesh()
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
        for (long i = 0; i < triangles.Length; i++)
        {
            triangles[i] = new Vector3(float.Parse(xyz[vertices.Length * 3 + 2 + i * 3]), float.Parse(xyz[vertices.Length * 3 + 2 + i * 3 + 1]), float.Parse(xyz[vertices.Length * 3 + 2 + i * 3 + 2]));
        }

        mesh = new Mesh();
        GetComponent<MeshFilter>().mesh = mesh;
        mesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;

        int[] indexArray = new int[triangles.Length * 3];
        int j = 0;
        for (int i = 0; i < triangles.Length; i++)
        {
            indexArray[j] = (int)triangles[i].x;
            indexArray[j + 1] = (int)triangles[i].y;
            indexArray[j + 2] = (int)triangles[i].z;

            j += 3;
        }

        mesh.Clear();
        mesh.vertices = vertices;
        mesh.triangles = indexArray;
        mesh.RecalculateNormals();
    }
}
