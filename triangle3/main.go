package main

import (
	"fmt"
	"log"
	"runtime"
	"strings"

	"github.com/go-gl/gl/all-core/gl"
	"github.com/go-gl/glfw/v3.1/glfw"
	"github.com/go-gl/mathgl/mgl32"
)

var vertexShader = `#version 400
layout(location = 0) in vec3 vertex_position;
layout(location = 1) in vec3 vertex_color;

out vec3 color;
uniform mat4 transform;

void main() {
  color = vertex_color;
  gl_Position = transform * vec4(vertex_position, 1.0);
}
` + "\x00"

var fragmentShader = `#version 400
in vec3 color;
out vec4 frag_color;

void main() {
  frag_color = vec4(color, 1.0);
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

	colors := []float32{
		1.0, 0.0, 0.0,
		0.0, 1.0, 0.0,
		0.0, 0.0, 1.0,
	}

	var pointVBO uint32
	gl.GenBuffers(1, &pointVBO)
	gl.BindBuffer(gl.ARRAY_BUFFER, pointVBO)
	gl.BufferData(gl.ARRAY_BUFFER, len(points)*4, gl.Ptr(points), gl.STATIC_DRAW)

	var colorVBO uint32
	gl.GenBuffers(1, &colorVBO)
	gl.BindBuffer(gl.ARRAY_BUFFER, colorVBO)
	gl.BufferData(gl.ARRAY_BUFFER, len(colors)*4, gl.Ptr(colors), gl.STATIC_DRAW)

	var vao uint32

	gl.GenVertexArrays(1, &vao)
	gl.BindVertexArray(vao)

	gl.BindBuffer(gl.ARRAY_BUFFER, pointVBO)
	gl.VertexAttribPointer(0, 3, gl.FLOAT, false, 0, gl.PtrOffset(0))
	gl.EnableVertexAttribArray(0)

	gl.BindBuffer(gl.ARRAY_BUFFER, colorVBO)
	gl.VertexAttribPointer(1, 3, gl.FLOAT, false, 0, gl.PtrOffset(0))
	gl.EnableVertexAttribArray(1)

	vs := gl.CreateShader(gl.VERTEX_SHADER)
	source, f := gl.Strs(vertexShader)
	defer f()
	gl.ShaderSource(vs, 1, source, nil)
	gl.CompileShader(vs)

	if err := checkShaderError(vertexShader, vs); err != nil {
		panic(err)
	}

	fs := gl.CreateShader(gl.FRAGMENT_SHADER)
	source, f = gl.Strs(fragmentShader)
	defer f()
	gl.ShaderSource(fs, 1, source, nil)
	gl.CompileShader(fs)

	if err := checkShaderError(fragmentShader, fs); err != nil {
		panic(err)
	}

	shader := gl.CreateProgram()
	gl.AttachShader(shader, fs)
	gl.AttachShader(shader, vs)
	gl.LinkProgram(shader)

	if err := checkProgramError(shader); err != nil {
		panic(err)
	}

	transformLoc := gl.GetUniformLocation(shader, gl.Str("transform\x00"))

	for !window.ShouldClose() {
		gl.Clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT)

		gl.UseProgram(shader)
		trans := mgl32.Ident4()
		trans = trans.Mul4(mgl32.Translate3D(0.5, -0.5, 0.0))
		trans = trans.Mul4(mgl32.HomogRotate3D(float32(glfw.GetTime()), mgl32.Vec3{0, 0, 1}))
		gl.UniformMatrix4fv(transformLoc, 1, false, &trans[0])

		gl.BindVertexArray(vao)
		gl.DrawArrays(gl.TRIANGLES, 0, 3)

		// Maintenance
		window.SwapBuffers()
		glfw.PollEvents()
	}
}

func checkShaderError(source string, s uint32) error {
	var status int32
	gl.GetShaderiv(s, gl.COMPILE_STATUS, &status)
	if status == gl.FALSE {
		var logLength int32
		gl.GetShaderiv(s, gl.INFO_LOG_LENGTH, &logLength)

		log := strings.Repeat("\x00", int(logLength+1))
		gl.GetShaderInfoLog(s, logLength, nil, gl.Str(log))

		return fmt.Errorf("failed to compile %v: %v", source, log)
	}
	return nil
}

func checkProgramError(s uint32) error {
	var status int32
	gl.GetProgramiv(s, gl.LINK_STATUS, &status)
	if status == gl.FALSE {
		var logLength int32
		gl.GetProgramiv(s, gl.INFO_LOG_LENGTH, &logLength)

		log := strings.Repeat("\x00", int(logLength+1))
		gl.GetProgramInfoLog(s, logLength, nil, gl.Str(log))

		return fmt.Errorf("failed to link: %v", log)
	}
	return nil
}
