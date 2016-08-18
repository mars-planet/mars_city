using UnityEngine;
using System.Collections;

private bool Load(string fileName)
 {
     // Handle any problems that might arise when reading the text
     try
     {
         string line;
         StreamReader theReader = new StreamReader(fileName, Encoding.Default);
      
         using (theReader)
         {
             do
             {
                 line = theReader.ReadLine();
                     
                 if (line != null)
                 {
                     string[] entries = line.Split(',');
                     if (entries.Length > 0)
                         print(entries);
                 }
             }
             while (line != null);
             theReader.Close();
             return true;
             }
         }
         catch (Exception e)
         {
             Console.WriteLine("{0}\n", e.Message);
             return false;
         }
     }
 }

public class ExampleClass : MonoBehaviour {
    public Material mat;
    private Vector3 startVertex;
    private Vector3 mousePos;
    void Update() {
        mousePos = Input.mousePosition;
        if (Input.GetKeyDown(KeyCode.Space))
            startVertex = new Vector3(mousePos.x / Screen.width, mousePos.y / Screen.height, 0);
        
    }
    void OnPostRender() {
        if (!mat) {
            Debug.LogError("Please Assign a material on the inspector");
            return;
        }
        GL.PushMatrix();
        mat.SetPass(0);
        GL.LoadOrtho();
        GL.Begin(GL.LINES);
        GL.Color(Color.red);
        GL.Vertex(startVertex);
        GL.Vertex(new Vector3(mousePos.x / Screen.width, mousePos.y / Screen.height, 0));
        GL.End();
        GL.PopMatrix();
    }
    void Example() {
        startVertex = new Vector3(0, 0, 0);
    }
}

  public static void DrawGLLine(Vector3 P1, Vector3 P2)
 {
     GL.PushMatrix();
     GL.LoadOrtho();
     GL.Begin(GL.LINES);
     GL.Color(Color.red);
     GL.Vertex(P1);
     GL.Vertex(P2);
     GL.End();
     GL.PopMatrix();        
 }
