package main

import (
	"bufio"
	"errors"
	"fmt"
	"image"
	"image/draw"
	_ "image/jpeg"
	_ "image/png"
	"log"
	"os"
	"runtime"
	"strconv"
	"strings"

	"github.com/disintegration/imaging"
	"github.com/go-gl/gl/all-core/gl"
	"github.com/go-gl/glfw/v3.1/glfw"
	"github.com/go-gl/mathgl/mgl32"
)

var vertexShader = `#version 400
layout(location = 0) in vec3 vertex_position;
layout(location = 1) in vec3 vertex_normal;
layout(location = 2) in vec2 vertex_uv;

out vec2 uv;
out vec3 normal;
out vec3 FragPos;
uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

void main() {
  uv = vertex_uv;
  normal = vertex_normal;
  FragPos = vec3(model * vec4(vertex_position, 1.0));
  gl_Position = projection * view * model * vec4(vertex_position, 1.0);
}
` + "\x00"

var fragmentShader = `#version 400
in vec2 uv;
in vec3 normal;
in vec3 FragPos;
out vec4 frag_colour;
uniform sampler2D tex;
uniform vec3 lightPos;

void main() {
  vec3 norm = normalize(normal);
  vec3 lightDir = normalize(lightPos - FragPos);  
  float diff = max(dot(norm, lightDir), 0.0);
  //vec3 diffuse = diff * lightColor;
  vec3 diffuse = diff * vec3(1,1,1);

  frag_colour = vec4(diffuse, 1) * texture(tex, uv);
}
` + "\x00"

var flatVertexShader = `#version 400
layout(location = 0) in vec3 vertex_position;
layout(location = 2) in vec2 vertex_uv;

out vec2 uv;
uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

void main() {
  uv = vertex_uv;
  gl_Position = projection * view * model * vec4(vertex_position, 1.0);
}
` + "\x00"

var flatFragmentShader = `#version 400
in vec2 uv;
out vec4 frag_colour;

uniform sampler2D tex;

void main() {
  frag_colour = texture(tex, uv);
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

	m, err := readObj("low-poly-fox-by-pixelmannen.obj")
	if err != nil {
		panic(err)
	}

	var vao uint32

	// positions
	// normals
	// uvs
	// vertex 1              vertex 2
	// x y z | r g b | u v | x y z | r g b | uv |
	// <------------------> 32 bytes for stride
	// 0       12      24 - offset
	// 8 floats per v1, v2, v3 vn1, vn2, vn3 uv1, uv2
	vertices := make([]float32, 0, len(m.FV)*8)
	for i := 0; i < len(m.FV); i++ {
		vecIndex := m.FV[i] - 1
		scale := float32(0.03)
		vertices = append(vertices, m.V[vecIndex*3+0]*scale)
		vertices = append(vertices, m.V[vecIndex*3+1]*scale)
		vertices = append(vertices, m.V[vecIndex*3+2]*scale)

		normIndex := m.FVN[i] - 1
		vertices = append(vertices, m.VN[normIndex*3+0])
		vertices = append(vertices, m.VN[normIndex*3+1])
		vertices = append(vertices, m.VN[normIndex*3+2])

		uvIndex := m.FVT[i] - 1
		vertices = append(vertices, m.VT[uvIndex*2+0])
		vertices = append(vertices, m.VT[uvIndex*2+1])
	}

	faces := len(m.FV)

	var vbo uint32
	gl.GenBuffers(1, &vbo)
	gl.BindBuffer(gl.ARRAY_BUFFER, vbo)
	gl.BufferData(gl.ARRAY_BUFFER, len(vertices)*4, gl.Ptr(vertices), gl.STATIC_DRAW)

	gl.GenVertexArrays(1, &vao)
	gl.BindVertexArray(vao)

	gl.BindBuffer(gl.ARRAY_BUFFER, vbo)
	gl.VertexAttribPointer(0, 3, gl.FLOAT, false, 32, gl.PtrOffset(0))
	gl.EnableVertexAttribArray(0)

	gl.BindBuffer(gl.ARRAY_BUFFER, vbo)
	gl.VertexAttribPointer(1, 3, gl.FLOAT, false, 32, gl.PtrOffset(12))
	gl.EnableVertexAttribArray(1)

	gl.BindBuffer(gl.ARRAY_BUFFER, vbo)
	gl.VertexAttribPointer(2, 2, gl.FLOAT, false, 32, gl.PtrOffset(24))
	gl.EnableVertexAttribArray(2)

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

	modelLoc := gl.GetUniformLocation(shader, gl.Str("model\x00"))
	viewLoc := gl.GetUniformLocation(shader, gl.Str("view\x00"))
	projectionLoc := gl.GetUniformLocation(shader, gl.Str("projection\x00"))

	lightPosLoc := gl.GetUniformLocation(shader, gl.Str("lightPos\x00"))

	texture, err := newTexture("texture.png")
	if err != nil {
		panic(err)
	}

	face, err := newTexture("pouting.png")
	if err != nil {
		panic(err)
	}

	cube := makeCube()
	flatShader := makeFlatShader()
	gl.UseProgram(flatShader)

	fsModelLoc := gl.GetUniformLocation(shader, gl.Str("model\x00"))
	fsViewLoc := gl.GetUniformLocation(shader, gl.Str("view\x00"))
	fsProjectionLoc := gl.GetUniformLocation(shader, gl.Str("projection\x00"))

	//view := mgl32.Translate3D(0, 0, -100)
	view := mgl32.Translate3D(0, 0, -6)

	lightPos := mgl32.Vec3{1.2, 1.0, 2.0}

	projection := mgl32.Perspective(mgl32.DegToRad(45.0), float32(windowWidth)/float32(windowHeight), 0.1, 100.0)
	gl.Enable(gl.DEPTH_TEST)

	for !window.ShouldClose() {
		gl.Clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT)

		gl.UseProgram(shader)

		model := mgl32.HomogRotate3D(
			float32(glfw.GetTime())*mgl32.DegToRad(50),
			mgl32.Vec3{0.5, 1, 0}.Normalize())

		gl.UniformMatrix4fv(modelLoc, 1, false, &model[0])
		gl.UniformMatrix4fv(viewLoc, 1, false, &view[0])
		gl.UniformMatrix4fv(projectionLoc, 1, false, &projection[0])

		gl.Uniform3fv(lightPosLoc, 1, &lightPos[0])
		//lightingShader.setVec3("objectColor", 1.0, 0.5, 0.31f);
		//lightingShader.setVec3("lightColor", 1.0, 1.0, 1.0f);
		//lightingShader.setVec3("lightPos", lightPos);

		gl.BindVertexArray(vao)

		gl.ActiveTexture(gl.TEXTURE0)
		gl.BindTexture(gl.TEXTURE_2D, texture)

		gl.DrawArrays(gl.TRIANGLES, 0, int32(faces))

		gl.BindVertexArray(cube)
		gl.ActiveTexture(gl.TEXTURE0)
		gl.BindTexture(gl.TEXTURE_2D, face)
		gl.DrawArrays(gl.TRIANGLES, 0, 36)

		model = mgl32.Translate3D(lightPos.X(), lightPos.Y(), lightPos.Z())
		model = model.Mul4(mgl32.Scale3D(0.1, 0.1, 0.1))
		gl.UseProgram(flatShader)
		gl.UniformMatrix4fv(fsModelLoc, 1, false, &model[0])
		gl.UniformMatrix4fv(fsViewLoc, 1, false, &view[0])
		gl.UniformMatrix4fv(fsProjectionLoc, 1, false, &projection[0])

		gl.BindVertexArray(cube)
		gl.DrawArrays(gl.TRIANGLES, 0, 36)

		// Maintenance
		window.SwapBuffers()
		glfw.PollEvents()
	}
}

type Obj struct {
	V  []float32 // v x y z
	VN []float32 // vn x y z
	VT []float32 // vt x y

	FV  []int // f v1 v2 v3
	FVN []int // f vn1 vn2 vn3
	FVT []int // f vt1 vt2 vt3
}

func readObj(file string) (Obj, error) {
	// Open the file for reading and check for errors.
	f, err := os.Open(file)
	if err != nil {
		return Obj{}, err
	}

	// Don't forget to close the file reader.
	defer f.Close()

	// Create a model to store stuff.
	model := Obj{}

	readVec3 := func(t string) (float32, float32, float32, error) {
		s := strings.Split(t, " ")
		if len(s) != 4 {
			return 0, 0, 0, errors.New("vec3: not enough elements: " + t)
		}
		x, err := strconv.ParseFloat(s[1], 32)
		if err != nil {
			return 0, 0, 0, err
		}
		y, err := strconv.ParseFloat(s[2], 32)
		if err != nil {
			return 0, 0, 0, err
		}
		z, err := strconv.ParseFloat(s[3], 32)
		if err != nil {
			return 0, 0, 0, err
		}
		return float32(x), float32(y), float32(z), nil
	}

	readVec2 := func(t string) (float32, float32, error) {
		s := strings.Split(t, " ")
		if len(s) != 3 {
			return 0, 0, errors.New("vec2: not enough elements: " + t)
		}
		x, err := strconv.ParseFloat(s[1], 32)
		if err != nil {
			return 0, 0, err
		}
		y, err := strconv.ParseFloat(s[2], 32)
		if err != nil {
			return 0, 0, err
		}
		return float32(x), float32(y), nil
	}

	scanner := bufio.NewScanner(f)
	for scanner.Scan() {
		t := scanner.Text()
		if strings.HasPrefix(t, "v ") {
			x, y, z, err := readVec3(t)
			if err != nil {
				return Obj{}, err
			}
			//fmt.Printf("v %f %f %f\n", v.X(), v.Y(), v.Z())
			model.V = append(model.V, x)
			model.V = append(model.V, y)
			model.V = append(model.V, z)
		} else if strings.HasPrefix(t, "vn ") {
			x, y, z, err := readVec3(t)
			if err != nil {
				return Obj{}, err
			}
			//fmt.Printf("vn %f %f %f\n", v.X(), v.Y(), v.Z())
			model.VN = append(model.VN, x)
			model.VN = append(model.VN, y)
			model.VN = append(model.VN, z)
		} else if strings.HasPrefix(t, "vt ") {
			x, y, err := readVec2(t)
			if err != nil {
				return Obj{}, err
			}
			//fmt.Printf("vt %f %f\n", v.X(), v.Y())
			model.VT = append(model.VT, x)
			model.VT = append(model.VT, y)
		} else if strings.HasPrefix(t, "f") {
			// f 0/0/0 0/0/0 0/0/0
			s := strings.Split(t, " ")
			if len(s) != 4 {
				return Obj{}, errors.New("face: not enough elements")
			}
			get3 := func(v string) (int, int, int, error) {
				s := strings.Split(v, "/")
				if len(s) != 3 {
					return 0, 0, 0, errors.New("not enough elements")
				}
				v1, err := strconv.Atoi(s[0])
				if err != nil {
					return 0, 0, 0, err
				}
				v2, err := strconv.Atoi(s[1])
				if err != nil {
					return 0, 0, 0, err
				}
				v3, err := strconv.Atoi(s[2])
				if err != nil {
					return 0, 0, 0, errors.New("not enough elements")
				}
				return v1, v2, v3, nil
			}
			v1, u1, n1, err := get3(s[1])
			if err != nil {
				return Obj{}, errors.New("face: not enough elements")
			}
			v2, u2, n2, err := get3(s[2])
			if err != nil {
				return Obj{}, errors.New("face: not enough elements")
			}
			v3, u3, n3, err := get3(s[3])
			if err != nil {
				return Obj{}, errors.New("face: not enough elements")
			}
			//fmt.Printf("f %d/%d/%d %d/%d/%d %d/%d/%d\n", v1, u1, n1, v2, u2, n2, v3, u3, n3)
			model.FVN = append(model.FVN, n1)
			model.FVN = append(model.FVN, n2)
			model.FVN = append(model.FVN, n3)

			model.FV = append(model.FV, v1)
			model.FV = append(model.FV, v2)
			model.FV = append(model.FV, v3)

			model.FVT = append(model.FVT, u1)
			model.FVT = append(model.FVT, u2)
			model.FVT = append(model.FVT, u3)
		}
	}

	if err := scanner.Err(); err != nil {
		log.Fatal(err)
	}
	return model, nil
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

func makeCube() uint32 {
	var vao uint32
	// positions | normals | uvs
	vertices := []float32{
		-1.0, -1.0, -1.0, 0.0, 0.0, -1.0, 0.0, 0.0,
		1.0, -1.0, -1.0, 0.0, 0.0, -1.0, 1.0, 0.0,
		1.0, 1.0, -1.0, 0.0, 0.0, -1.0, 1.0, 1.0,
		1.0, 1.0, -1.0, 0.0, 0.0, -1.0, 1.0, 1.0,
		-1.0, 1.0, -1.0, 0.0, 0.0, -1.0, 0.0, 1.0,
		-1.0, -1.0, -1.0, 0.0, 0.0, -1.0, 0.0, 0.0,

		-1.0, -1.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0,
		1.0, -1.0, 1.0, 0.0, 0.0, 1.0, 1.0, 0.0,
		1.0, 1.0, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0,
		1.0, 1.0, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0,
		-1.0, 1.0, 1.0, 0.0, 0.0, 1.0, 0.0, 1.0,
		-1.0, -1.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0,

		-1.0, 1.0, 1.0, -1.0, 0.0, 0.0, 1.0, 0.0,
		-1.0, 1.0, -1.0, -1.0, 0.0, 0.0, 1.0, 1.0,
		-1.0, -1.0, -1.0, -1.0, 0.0, 0.0, 0.0, 1.0,
		-1.0, -1.0, -1.0, -1.0, 0.0, 0.0, 0.0, 1.0,
		-1.0, -1.0, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0,
		-1.0, 1.0, 1.0, -1.0, 0.0, 0.0, 1.0, 0.0,

		1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 1.0, 0.0,
		1.0, 1.0, -1.0, 1.0, 0.0, 0.0, 1.0, 1.0,
		1.0, -1.0, -1.0, 1.0, 0.0, 0.0, 0.0, 1.0,
		1.0, -1.0, -1.0, 1.0, 0.0, 0.0, 0.0, 1.0,
		1.0, -1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0,
		1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 1.0, 0.0,

		-1.0, -1.0, -1.0, 0.0, -1.0, 0.0, 0.0, 1.0,
		1.0, -1.0, -1.0, 0.0, -1.0, 0.0, 1.0, 1.0,
		1.0, -1.0, 1.0, 0.0, -1.0, 0.0, 1.0, 0.0,
		1.0, -1.0, 1.0, 0.0, -1.0, 0.0, 1.0, 0.0,
		-1.0, -1.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0,
		-1.0, -1.0, -1.0, 0.0, -1.0, 0.0, 0.0, 1.0,

		-1.0, 1.0, -1.0, 0.0, 1.0, 0.0, 0.0, 1.0,
		1.0, 1.0, -1.0, 0.0, 1.0, 0.0, 1.0, 1.0,
		1.0, 1.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0,
		1.0, 1.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0,
		-1.0, 1.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0,
		-1.0, 1.0, -1.0, 0.0, 1.0, 0.0, 0.0, 1.0,
	}
	var vbo uint32
	gl.GenBuffers(1, &vbo)
	gl.BindBuffer(gl.ARRAY_BUFFER, vbo)
	gl.BufferData(gl.ARRAY_BUFFER, len(vertices)*4, gl.Ptr(vertices), gl.STATIC_DRAW)

	gl.GenVertexArrays(1, &vao)
	gl.BindVertexArray(vao)

	gl.BindBuffer(gl.ARRAY_BUFFER, vbo)
	gl.VertexAttribPointer(0, 3, gl.FLOAT, false, 32, gl.PtrOffset(0))
	gl.EnableVertexAttribArray(0)

	gl.BindBuffer(gl.ARRAY_BUFFER, vbo)
	gl.VertexAttribPointer(1, 3, gl.FLOAT, false, 32, gl.PtrOffset(12))
	gl.EnableVertexAttribArray(1)

	gl.BindBuffer(gl.ARRAY_BUFFER, vbo)
	gl.VertexAttribPointer(2, 2, gl.FLOAT, false, 32, gl.PtrOffset(24))
	gl.EnableVertexAttribArray(2)

	return vao
}

func makeFlatShader() uint32 {
	vs := gl.CreateShader(gl.VERTEX_SHADER)
	source, f := gl.Strs(flatVertexShader)
	defer f()
	gl.ShaderSource(vs, 1, source, nil)
	gl.CompileShader(vs)

	if err := checkShaderError(flatVertexShader, vs); err != nil {
		panic(err)
	}

	fs := gl.CreateShader(gl.FRAGMENT_SHADER)
	source, f = gl.Strs(flatFragmentShader)
	defer f()
	gl.ShaderSource(fs, 1, source, nil)
	gl.CompileShader(fs)

	if err := checkShaderError(flatFragmentShader, fs); err != nil {
		panic(err)
	}

	shader := gl.CreateProgram()
	gl.AttachShader(shader, fs)
	gl.AttachShader(shader, vs)
	gl.LinkProgram(shader)

	if err := checkProgramError(shader); err != nil {
		panic(err)
	}
	return shader
}
