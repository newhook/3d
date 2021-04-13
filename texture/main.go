package main

import (
	"fmt"
	"image"
	"image/draw"
	_ "image/jpeg"
	_ "image/png"
	"log"
	"os"
	"runtime"
	"strings"

	"github.com/disintegration/imaging"
	"github.com/go-gl/gl/all-core/gl"
	"github.com/go-gl/glfw/v3.1/glfw"
	"github.com/go-gl/mathgl/mgl32"
)

var vertexShader = `#version 400
layout(location = 0) in vec3 vertex_position;
layout(location = 1) in vec3 vertex_colour;
layout(location = 2) in vec2 vertex_uv;

out vec3 colour;
out vec2 uv;
uniform mat4 transform;

void main() {
  colour = vertex_colour;
  uv = vertex_uv;
  gl_Position = transform * vec4(vertex_position, 1.0);
}
` + "\x00"

var fragmentShader = `#version 400
in vec3 colour;
in vec2 uv;
out vec4 frag_colour;

uniform sampler2D tex;
uniform sampler2D tex2;

void main() {
  frag_colour = mix(texture(tex, uv), texture(tex2, uv), 0.2);
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

	var vao uint32
	if false {
		// Define points, colors and uvs in separate VBO.
		points := []float32{
			0.5, 0.5, 0.0,
			0.5, -0.5, 0.0,
			-0.5, -0.5, 0.0,
			-0.5, 0.5, 0.0,
		}

		colors := []float32{
			1.0, 0.0, 0.0,
			0.0, 1.0, 0.0,
			0.0, 0.0, 1.0,
			1.0, 1.0, 0.0,
		}

		uvs := []float32{
			1.0, 1.0,
			1.0, 0.0,
			0.0, 0.0,
			0.0, 1.0,
		}

		indices := []int32{
			0, 1, 3, // first triangle
			1, 2, 3, // second triangle
		}
		var ebo uint32
		gl.GenBuffers(1, &ebo)
		gl.BindBuffer(gl.ELEMENT_ARRAY_BUFFER, ebo)
		gl.BufferData(gl.ELEMENT_ARRAY_BUFFER, len(indices)*4, gl.Ptr(indices), gl.STATIC_DRAW)

		var pointVBO uint32
		gl.GenBuffers(1, &pointVBO)
		gl.BindBuffer(gl.ARRAY_BUFFER, pointVBO)
		gl.BufferData(gl.ARRAY_BUFFER, len(points)*4, gl.Ptr(points), gl.STATIC_DRAW)

		var colorVBO uint32
		gl.GenBuffers(1, &colorVBO)
		gl.BindBuffer(gl.ARRAY_BUFFER, colorVBO)
		gl.BufferData(gl.ARRAY_BUFFER, len(colors)*4, gl.Ptr(colors), gl.STATIC_DRAW)

		var uvVBO uint32
		gl.GenBuffers(1, &uvVBO)
		gl.BindBuffer(gl.ARRAY_BUFFER, uvVBO)
		gl.BufferData(gl.ARRAY_BUFFER, len(uvs)*4, gl.Ptr(uvs), gl.STATIC_DRAW)

		gl.GenVertexArrays(1, &vao)
		gl.BindVertexArray(vao)

		gl.BindBuffer(gl.ELEMENT_ARRAY_BUFFER, ebo)

		gl.BindBuffer(gl.ARRAY_BUFFER, pointVBO)
		gl.VertexAttribPointer(0, 3, gl.FLOAT, false, 0, gl.PtrOffset(0))
		gl.EnableVertexAttribArray(0)

		gl.BindBuffer(gl.ARRAY_BUFFER, colorVBO)
		gl.VertexAttribPointer(1, 3, gl.FLOAT, false, 0, gl.PtrOffset(0))
		gl.EnableVertexAttribArray(1)

		gl.BindBuffer(gl.ARRAY_BUFFER, uvVBO)
		gl.VertexAttribPointer(2, 2, gl.FLOAT, false, 0, gl.PtrOffset(0))
		gl.EnableVertexAttribArray(2)
	} else {
		// positions
		// colors
		// uvs
		// vertex 1              vertex 2
		// x y z | r g b | u v | x y z | r g b | uv |
		// <------------------> 32 bytes for stride
		// 0       12      24 - offset
		vertices := []float32{
			0.5, 0.5, 0.0, // positions
			1.0, 0.0, 0.0, // colors
			1.0, 1.0, // texture coords
			0.5, -0.5, 0.0,
			0.0, 1.0, 0.0,
			1.0, 0.0,
			-0.5, -0.5, 0.0,
			0.0, 0.0, 1.0,
			0.0, 0.0,
			-0.5, 0.5, 0.0,
			1.0, 1.0, 0.0,
			0.0, 1.0,
		}
		indices := []int32{
			0, 1, 3, // first triangle
			1, 2, 3, // second triangle
		}

		var ebo uint32
		gl.GenBuffers(1, &ebo)
		gl.BindBuffer(gl.ELEMENT_ARRAY_BUFFER, ebo)
		gl.BufferData(gl.ELEMENT_ARRAY_BUFFER, len(indices)*4, gl.Ptr(indices), gl.STATIC_DRAW)

		var vbo uint32
		gl.GenBuffers(1, &vbo)
		gl.BindBuffer(gl.ARRAY_BUFFER, vbo)
		gl.BufferData(gl.ARRAY_BUFFER, len(vertices)*4, gl.Ptr(vertices), gl.STATIC_DRAW)

		gl.GenVertexArrays(1, &vao)
		gl.BindVertexArray(vao)

		gl.BindBuffer(gl.ELEMENT_ARRAY_BUFFER, ebo)

		gl.BindBuffer(gl.ARRAY_BUFFER, vbo)
		gl.VertexAttribPointer(0, 3, gl.FLOAT, false, 32, gl.PtrOffset(0))
		gl.EnableVertexAttribArray(0)

		gl.BindBuffer(gl.ARRAY_BUFFER, vbo)
		gl.VertexAttribPointer(1, 3, gl.FLOAT, false, 32, gl.PtrOffset(12))
		gl.EnableVertexAttribArray(1)

		gl.BindBuffer(gl.ARRAY_BUFFER, vbo)
		gl.VertexAttribPointer(2, 2, gl.FLOAT, false, 32, gl.PtrOffset(24))
		gl.EnableVertexAttribArray(2)
	}

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

	gl.UseProgram(shader)
	gl.Uniform1i(gl.GetUniformLocation(shader, gl.Str("tex\x00")), 0)
	gl.Uniform1i(gl.GetUniformLocation(shader, gl.Str("tex2\x00")), 1)

	transformLoc := gl.GetUniformLocation(shader, gl.Str("transform\x00"))

	texture, err := newTexture("container.jpg")
	if err != nil {
		panic(err)
	}

	texture2, err := newTexture("pouting.png")
	if err != nil {
		panic(err)
	}

	for !window.ShouldClose() {
		gl.Clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT)

		gl.UseProgram(shader)
		trans := mgl32.Ident4()
		trans = trans.Mul4(mgl32.Translate3D(0.5, -0.5, 0.0))
		trans = trans.Mul4(mgl32.HomogRotate3D(float32(glfw.GetTime()), mgl32.Vec3{0, 0, 1}))
		gl.UniformMatrix4fv(transformLoc, 1, false, &trans[0])

		gl.BindVertexArray(vao)

		gl.ActiveTexture(gl.TEXTURE0)
		gl.BindTexture(gl.TEXTURE_2D, texture)
		gl.ActiveTexture(gl.TEXTURE1)
		gl.BindTexture(gl.TEXTURE_2D, texture2)

		//gl.DrawArrays(gl.TRIANGLES, 0, 6)
		gl.DrawElements(gl.TRIANGLES, 6, gl.UNSIGNED_INT, nil)

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

func newTexture(file string) (uint32, error) {
	imgFile, err := os.Open(file)
	if err != nil {
		return 0, fmt.Errorf("texture %q not found on disk: %v", file, err)
	}
	img, _, err := image.Decode(imgFile)
	if err != nil {
		return 0, err
	}
	img = imaging.FlipV(img)

	rgba := image.NewRGBA(img.Bounds())
	if rgba.Stride != rgba.Rect.Size().X*4 {
		return 0, fmt.Errorf("unsupported stride")
	}
	draw.Draw(rgba, rgba.Bounds(), img, image.Point{0, 0}, draw.Src)

	var texture uint32
	gl.GenTextures(1, &texture)
	gl.ActiveTexture(gl.TEXTURE0)
	gl.BindTexture(gl.TEXTURE_2D, texture)
	gl.TexParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR)
	gl.TexParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR)
	gl.TexParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE)
	gl.TexParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE)
	gl.TexImage2D(
		gl.TEXTURE_2D,
		0,
		gl.RGBA,
		int32(rgba.Rect.Size().X),
		int32(rgba.Rect.Size().Y),
		0,
		gl.RGBA,
		gl.UNSIGNED_BYTE,
		gl.Ptr(rgba.Pix))

	return texture, nil
}
