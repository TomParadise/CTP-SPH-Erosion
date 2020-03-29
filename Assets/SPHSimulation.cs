using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;

public class SPHSimulation : MonoBehaviour
{
    [SerializeField] private GameObject sphere;
    private StreamReader File;
    private string text;
    private GameObject[] particles;
    private int frame = 0;
    
    // Start is called before the first frame update
    void Start()
    {
        particles = new GameObject[400];
        for(int i = 0; i<particles.Length; i++)
        {
            particles[i] = Instantiate(sphere);
        }
    }

    // Update is called once per frame
    void Update()
    {
        if (frame < 100)
        {
            File = new StreamReader("Assets/Positions/DamBreak" + frame.ToString() + ".txt");
            text = File.ReadToEnd();
            long j = 0;
            for (long i = 0; i < particles.Length; i++)
            {
                string[] xyz = text.Split(',');
                //Debug.Log(i + " x " + xyz[j] + " y " + xyz[j+1] + " z " + xyz[j+2]);
                particles[i].transform.position = new Vector3(float.Parse(xyz[j]), float.Parse(xyz[j + 1]), float.Parse(xyz[j + 2]));
                particles[i].name = "" + i;
                j += 3;
            }
            //Debug.Log(frame);
            frame++;
        }
        else
        {
            frame = 1;
        }
    }
}
