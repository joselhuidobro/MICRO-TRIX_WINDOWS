import io.swagger.v3.oas.annotations.Operation;

@RestController
@RequestMapping("/api/v1")
class HelloController {

  @Operation(summary = "Saludo básico", description = "Devuelve un saludo simple")
  @GetMapping("/hello")
  public Map<String,String> hello() {
    return Map.of("message", "Hello, World!");
  }
}
			
