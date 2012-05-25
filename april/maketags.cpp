#include "TagFamily.h"
#include <cairo/cairo.h>
#include <cairo/cairo-pdf.h>

#include <iostream>

int main(int argc, char** argv) {

  /*
  if (argc != 3) {
    std::cerr << "Usage: " << argv[0] << " FAMILY OUTPUTDIR\n";
    std::cerr << "Known tag families:";
    TagFamily::StringArray known = TagFamily::families();
    for (size_t i=0; i<known.size(); ++i) { std::cerr << " " << known[i]; }
    std::cerr << "\n";
    return 1;
  }

  TagFamily family(argv[1]);

  family.writeAllImagesPostScript(argv[2] + std::string("/") + argv[1] + ".ps");
  */

  cairo_surface_t *surface;
  cairo_t *cr;

  surface = cairo_pdf_surface_create("pdffile.pdf", 504, 648);
  cr = cairo_create(surface);

  cairo_set_source_rgb(cr, 0, 0, 0);


  cairo_select_font_face (cr, "Sans", CAIRO_FONT_SLANT_NORMAL,
      CAIRO_FONT_WEIGHT_NORMAL);
  cairo_set_font_size (cr, 40.0);

  cairo_move_to(cr, 300.0, 100.0);
  cairo_show_text(cr, "Disziplin ist Macht.");
  
  cairo_move_to (cr, 50.0, 75.0);
  cairo_line_to (cr, 200.0, 75.0);
  
  cairo_move_to (cr, 50.0, 125.0);
  cairo_line_to (cr, 200.0, 125.0);
  
  cairo_move_to (cr, 50.0, 175.0);
  cairo_line_to (cr, 200.0, 175.0);
  
  cairo_set_line_width (cr, 30.0);
  cairo_set_line_cap (cr, CAIRO_LINE_CAP_ROUND);
  cairo_stroke (cr);

  cairo_show_page(cr);

  cairo_surface_destroy(surface);
  cairo_destroy(cr);

  return 0;
}
