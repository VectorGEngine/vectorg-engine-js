@echo off
setlocal

set "ENGINE_JS_DIR=%~dp0"
set "BUILD_DIR=%ENGINE_JS_DIR%builds\vectorg-engine-3d"
set "CARGO_EXE=%USERPROFILE%\.cargo\bin\cargo.exe"
set "LOCAL_WASM_PACK=%ENGINE_JS_DIR%node_modules\.bin\wasm-pack.cmd"
set "LOCAL_TSC=%ENGINE_JS_DIR%node_modules\.bin\tsc.cmd"
set "BUILD_TEMP=%ENGINE_JS_DIR%.windows-build-temp"
set "PATH=%USERPROFILE%\.cargo\bin;%PATH%"

if not exist "%CARGO_EXE%" (
	echo ERROR: cargo.exe was not found at "%CARGO_EXE%".
	exit /b 1
)

if not exist "%LOCAL_WASM_PACK%" (
	echo ERROR: wasm-pack was not found at "%LOCAL_WASM_PACK%".
	exit /b 1
)

if not exist "%LOCAL_TSC%" (
	echo ERROR: TypeScript compiler was not found at "%LOCAL_TSC%".
	exit /b 1
)

if not exist "%BUILD_TEMP%" mkdir "%BUILD_TEMP%"
if errorlevel 1 (
	echo ERROR: Could not create "%BUILD_TEMP%".
	exit /b 1
)
set "TEMP=%BUILD_TEMP%"
set "TMP=%BUILD_TEMP%"

echo [1/3] Preparing dim3 non-deterministic project...
pushd "%ENGINE_JS_DIR%"
"%CARGO_EXE%" run -p prepare_builds -- -d dim3 -f non-deterministic
if errorlevel 1 (
	popd
	echo ERROR: Project preparation failed.
	exit /b 1
)
popd

echo [2/3] Building WebAssembly...
pushd "%BUILD_DIR%"
"%CARGO_EXE%" clean
if errorlevel 1 (
	popd
	echo ERROR: Cargo clean failed.
	exit /b 1
)
call "%LOCAL_WASM_PACK%" build
if errorlevel 1 (
	popd
	echo ERROR: WebAssembly build failed.
	exit /b 1
)

echo [3/3] Building the public JavaScript and TypeScript package...
node -e "const fs = require('node:fs'); const source = '../../src.ts'; const destination = 'pkg/src'; fs.rmSync(destination, { recursive: true, force: true }); fs.cpSync(source, destination, { recursive: true }); fs.rmSync('pkg/raw.ts', { force: true }); fs.writeFileSync('pkg/src/raw.ts', 'export * from \"./vectorg_engine_wasm3d\";\n'); for (const path of fs.readdirSync(destination, { recursive: true }).map(entry => destination + '/' + entry).filter(path => fs.statSync(path).isFile())) { const lines = fs.readFileSync(path, 'utf8').split(/(?<=\n)/); let skipping = false; fs.writeFileSync(path, lines.filter(line => { if (line.includes('#if DIM2')) { skipping = true; return false; } if (skipping && line.includes('#endif')) { skipping = false; return false; } return !skipping; }).join('')); }"
if errorlevel 1 (
	popd
	echo ERROR: TypeScript source preparation failed.
	exit /b 1
)
call "%LOCAL_TSC%" --project "tsconfig.json"
if errorlevel 1 (
	popd
	echo ERROR: TypeScript package build failed.
	exit /b 1
)

node -e "const fs = require('node:fs'); const path = 'pkg/package.json'; const pkg = JSON.parse(fs.readFileSync(path, 'utf8')); pkg.name = '@vectorg/vectorg-engine-3d'; pkg.files = ['*']; pkg.module = 'vectorg-engine.js'; pkg.types = 'vectorg-engine.d.ts'; pkg.sideEffects = ['./*.js']; fs.writeFileSync(path, JSON.stringify(pkg, null, 2) + '\n');"
if errorlevel 1 (
	popd
	echo ERROR: Package metadata normalization failed.
	exit /b 1
)
if exist "pkg\.gitignore" del /q "pkg\.gitignore"
copy /y "NOTICE" "pkg\NOTICE" >nul
if errorlevel 1 (
	popd
	echo ERROR: Copying the package notice failed.
	exit /b 1
)
popd

if exist "%BUILD_TEMP%" rmdir /s /q "%BUILD_TEMP%"

echo.
echo Engine package build completed: "%BUILD_DIR%\pkg"
exit /b 0
