package main

import (
	"fmt"
	"log"
	"runtime"

	"github.com/go-gl/gl/all-core/gl"
	"github.com/go-gl/glfw/v3.1/glfw"
)

var vertexShader = `#version 400
in vec3 vp;
void main() {
  gl_Position = vec4(vp, 1.0);
}
` + "\x00"

var fragmentShader = `#version 400
out vec4 frag_colour;
void main() {
  frag_colour = vec4(0.5, 0.0, 0.5, 1.0);
}
` + "\x00"

const windowWidth = 800
const windowHeight = 600

func init() {
	// GLFW event handling must run on the main OS thread
	runtime.LockOSThread()
}

func main() {
	if err := glfw.Init(); err != nil {
		log.Fatalln("failed to initialize glfw:", err)
	}
	defer glfw.Terminate()

	glfw.WindowHint(glfw.Resizable, glfw.False)
	glfw.WindowHint(glfw.ContextVersionMajor, 4)
	glfw.WindowHint(glfw.ContextVersionMinor, 1)
	glfw.WindowHint(glfw.OpenGLProfile, glfw.OpenGLCoreProfile)
	glfw.WindowHint(glfw.OpenGLForwardCompatible, glfw.True)
	window, err := glfw.CreateWindow(windowWidth, windowHeight, "Cube", nil, nil)
	if err != nil {
		panic(err)
	}
	window.MakeContextCurrent()

	// Initialize Glow
	if err := gl.Init(); err != nil {
		panic(err)
	}

	version := gl.GoStr(gl.GetString(gl.VERSION))
	fmt.Println("OpenGL version", version)

	points := []float32{
		0.0, 0.5, 0.0,
		0.5, -0.5, 0.0,
		-0.5, -0.5, 0.0,
	}

	var pointVBO uint32
	gl.GenBuffers(1, &pointVBO)
	gl.BindBuffer(gl.ARRAY_BUFFER, pointVBO)
	gl.BufferData(gl.ARRAY_BUFFER, len(points)*4, gl.Ptr(points), gl.STATIC_DRAW)

	var vao uint32

	gl.GenVertexArrays(1, &vao)
	gl.BindVertexArray(vao)

	gl.BindBuffer(gl.ARRAY_BUFFER, pointVBO)
	gl.VertexAttribPointer(0, 3, gl.FLOAT, false, 0, gl.PtrOffset(0))
	gl.EnableVertexAttribArray(0)

	vs := gl.CreateShader(gl.VERTEX_SHADER)
	source, _ := gl.Strs(vertexShader)
	gl.ShaderSource(vs, 1, source, nil)
	gl.CompileShader(vs)

	fs := gl.CreateShader(gl.FRAGMENT_SHADER)
	source, _ = gl.Strs(fragmentShader)
	gl.ShaderSource(fs, 1, source, nil)
	gl.CompileShader(fs)

	shader := gl.CreateProgram()
	gl.AttachShader(shader, fs)
	gl.AttachShader(shader, vs)
	gl.LinkProgram(shader)

	for !window.ShouldClose() {
		gl.Clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT)

		gl.UseProgram(shader)
		gl.BindVertexArray(vao)
		gl.DrawArrays(gl.TRIANGLES, 0, 3)

		// Maintenance
		window.SwapBuffers()
		glfw.PollEvents()
	}
}
