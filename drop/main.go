package main

import (
	"fmt"
	"image"
	"image/draw"
	_ "image/jpeg"
	_ "image/png"
	"log"
	"math"
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
layout(location = 1) in vec2 vertex_uv;

out vec2 uv;
uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

void main() {
  uv = vertex_uv;
  gl_Position = projection * view * model * vec4(vertex_position, 1.0);
}
` + "\x00"

var fragmentShader = `#version 400
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
	vertices := []float32{
		//  X, Y, Z, U, V
		// Bottom
		-1.0, -1.0, -1.0, 0.0, 0.0,
		1.0, -1.0, -1.0, 1.0, 0.0,
		-1.0, -1.0, 1.0, 0.0, 1.0,
		1.0, -1.0, -1.0, 1.0, 0.0,
		1.0, -1.0, 1.0, 1.0, 1.0,
		-1.0, -1.0, 1.0, 0.0, 1.0,

		// Top
		-1.0, 1.0, -1.0, 0.0, 0.0,
		-1.0, 1.0, 1.0, 0.0, 1.0,
		1.0, 1.0, -1.0, 1.0, 0.0,
		1.0, 1.0, -1.0, 1.0, 0.0,
		-1.0, 1.0, 1.0, 0.0, 1.0,
		1.0, 1.0, 1.0, 1.0, 1.0,

		// Front
		-1.0, -1.0, 1.0, 1.0, 0.0,
		1.0, -1.0, 1.0, 0.0, 0.0,
		-1.0, 1.0, 1.0, 1.0, 1.0,
		1.0, -1.0, 1.0, 0.0, 0.0,
		1.0, 1.0, 1.0, 0.0, 1.0,
		-1.0, 1.0, 1.0, 1.0, 1.0,

		// Back
		-1.0, -1.0, -1.0, 0.0, 0.0,
		-1.0, 1.0, -1.0, 0.0, 1.0,
		1.0, -1.0, -1.0, 1.0, 0.0,
		1.0, -1.0, -1.0, 1.0, 0.0,
		-1.0, 1.0, -1.0, 0.0, 1.0,
		1.0, 1.0, -1.0, 1.0, 1.0,

		// Left
		-1.0, -1.0, 1.0, 0.0, 1.0,
		-1.0, 1.0, -1.0, 1.0, 0.0,
		-1.0, -1.0, -1.0, 0.0, 0.0,
		-1.0, -1.0, 1.0, 0.0, 1.0,
		-1.0, 1.0, 1.0, 1.0, 1.0,
		-1.0, 1.0, -1.0, 1.0, 0.0,

		// Right
		1.0, -1.0, 1.0, 1.0, 1.0,
		1.0, -1.0, -1.0, 1.0, 0.0,
		1.0, 1.0, -1.0, 0.0, 0.0,
		1.0, -1.0, 1.0, 1.0, 1.0,
		1.0, 1.0, -1.0, 0.0, 0.0,
		1.0, 1.0, 1.0, 0.0, 1.0,
	}
	var vbo uint32
	gl.GenBuffers(1, &vbo)
	gl.BindBuffer(gl.ARRAY_BUFFER, vbo)
	gl.BufferData(gl.ARRAY_BUFFER, len(vertices)*4, gl.Ptr(vertices), gl.STATIC_DRAW)

	gl.GenVertexArrays(1, &vao)
	gl.BindVertexArray(vao)

	gl.BindBuffer(gl.ARRAY_BUFFER, vbo)
	gl.VertexAttribPointer(0, 3, gl.FLOAT, false, 20, gl.PtrOffset(0))
	gl.EnableVertexAttribArray(0)

	gl.BindBuffer(gl.ARRAY_BUFFER, vbo)
	gl.VertexAttribPointer(1, 2, gl.FLOAT, false, 20, gl.PtrOffset(12))
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

	gl.UseProgram(shader)
	gl.Uniform1i(gl.GetUniformLocation(shader, gl.Str("tex\x00")), 0)
	gl.Uniform1i(gl.GetUniformLocation(shader, gl.Str("tex2\x00")), 1)

	modelLoc := gl.GetUniformLocation(shader, gl.Str("model\x00"))
	viewLoc := gl.GetUniformLocation(shader, gl.Str("view\x00"))
	projectionLoc := gl.GetUniformLocation(shader, gl.Str("projection\x00"))

	texture, err := newTexture("container.jpg")
	if err != nil {
		panic(err)
	}

	texture2, err := newTexture("pouting.png")
	if err != nil {
		panic(err)
	}

	cameraPos := mgl32.Vec3{0.0, 0.0, 3.0}
	cameraFront := mgl32.Vec3{0.0, 0.0, -1.0}
	cameraUp := mgl32.Vec3{0.0, 1.0, 0.0}
	baseCameraSpeed := 2.5
	fov := float32(45.0)

	gl.Enable(gl.DEPTH_TEST)

	firstMouse := true
	yaw := float32(-90.0) // yaw is initialized to -90.0 degrees since a yaw of 0.0 results in a direction vector pointing to the right so we initially rotate a bit to the left.
	pitch := float32(0.0)
	lastX := float32(800.0 / 2.0)
	lastY := float32(600.0 / 2.0)

	window.SetCursorPosCallback(func(w *glfw.Window, xp float64, yp float64) {
		xpos := float32(xp)
		ypos := float32(yp)
		if firstMouse {
			lastX = xpos
			lastY = ypos
			firstMouse = false
		}

		xoffset := xpos - lastX
		yoffset := lastY - ypos // reversed since y-coordinates go from bottom to top
		lastX = xpos
		lastY = ypos

		sensitivity := float32(0.01) // change this value to your liking
		xoffset *= sensitivity
		yoffset *= sensitivity

		yaw += xoffset
		pitch += yoffset

		// make sure that when pitch is out of bounds, screen doesn't get flipped
		if pitch > 89.0 {
			pitch = 89.0
		}
		if pitch < -89.0 {
			pitch = -89.0
		}

		front := mgl32.Vec3{
			cos(mgl32.DegToRad(yaw)) * cos(mgl32.DegToRad(pitch)),
			sin(mgl32.DegToRad(pitch)),
			sin(mgl32.DegToRad(yaw)) * cos(mgl32.DegToRad(pitch))}
		cameraFront = front.Normalize()
	})

	window.SetScrollCallback(func(w *glfw.Window, xo float64, yo float64) {
		fov -= float32(yo)
		if fov < 1.0 {
			fov = 1.0
		}
		if fov > 45.0 {
			fov = 45.0
		}
	})
	angles := []float32{}
	drops := []mgl32.Vec3{}
	window.SetMouseButtonCallback(func(w *glfw.Window, button glfw.MouseButton, action glfw.Action, mod glfw.ModifierKey) {
		if button == glfw.MouseButton1 {
			drops = append(drops, cameraPos)
			angles = append(angles, 0.0)
		}
	})
	window.SetInputMode(glfw.CursorMode, glfw.CursorDisabled)

	var (
		deltaTime float64
		lastFrame float64
	)
	for !window.ShouldClose() {

		currentFrame := glfw.GetTime()
		deltaTime = currentFrame - lastFrame
		lastFrame = currentFrame

		cameraSpeed := float32(baseCameraSpeed * deltaTime)

		if state := window.GetKey(glfw.KeyW); state == glfw.Press {
			cameraPos = cameraPos.Add(cameraFront.Mul(cameraSpeed))
		}
		if state := window.GetKey(glfw.KeyS); state == glfw.Press {
			cameraPos = cameraPos.Sub(cameraFront.Mul(cameraSpeed))
		}
		if state := window.GetKey(glfw.KeyA); state == glfw.Press {
			cameraPos = cameraPos.Sub(cameraFront.Cross(cameraUp).Normalize().Mul(cameraSpeed))
		}
		if state := window.GetKey(glfw.KeyD); state == glfw.Press {
			cameraPos = cameraPos.Add(cameraFront.Cross(cameraUp).Normalize().Mul(cameraSpeed))
		}

		gl.Clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT)

		gl.UseProgram(shader)

		view := mgl32.LookAtV(cameraPos, cameraPos.Add(cameraFront), cameraUp)
		gl.UniformMatrix4fv(viewLoc, 1, false, &view[0])
		projection := mgl32.Perspective(mgl32.DegToRad(fov), float32(windowWidth)/float32(windowHeight), 0.1, 100.0)
		gl.UniformMatrix4fv(projectionLoc, 1, false, &projection[0])

		gl.BindVertexArray(vao)

		gl.ActiveTexture(gl.TEXTURE0)
		gl.BindTexture(gl.TEXTURE_2D, texture)
		gl.ActiveTexture(gl.TEXTURE1)
		gl.BindTexture(gl.TEXTURE_2D, texture2)

		for i := range drops {
			angles[i] += float32(deltaTime)
			drops[i] = drops[i].Sub(mgl32.Vec3{0, 0, 5 * float32(deltaTime)})

			model := mgl32.Translate3D(drops[i].X(), drops[i].Y(), drops[i].Z())
			model = model.Mul4(mgl32.HomogRotate3D(float32(angles[i]), mgl32.Vec3{0.5, 1, 0}.Normalize()))

			gl.UniformMatrix4fv(modelLoc, 1, false, &model[0])
			gl.DrawArrays(gl.TRIANGLES, 0, 36)
		}

		// Maintenance
		window.SwapBuffers()
		glfw.PollEvents()

	}
}

func sin(v float32) float32 {
	return float32(math.Sin(float64(v)))
}
func cos(v float32) float32 {
	return float32(math.Cos(float64(v)))
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
