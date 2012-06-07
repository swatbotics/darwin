#include "TagFamily.h"
#include <cairo/cairo.h>
#include <cairo/cairo-pdf.h>

#include <stdio.h>
#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <ctype.h>

struct Unit {
  std::string name;
  double points;
};

static const Unit units[] = {
  { "pt", 1 },
  { "in", 72 },
  { "ft", 72*12 },
  { "cm", 28.346456693 },
  { "mm", 2.8346456693 },
  { "", 0 }
};


struct PaperSize {
  std::string name;
  double width_points;
  double height_points;
};

static const PaperSize papers[] = {
  { "letter", 612, 792 },
  { "a4", 595, 842 },
  { "", 0 }
};


double parse_unit(std::istream& istr) {

  double rval;

  if (!(istr>>rval)) {
    std::cerr << "error: expected number\n";
    exit(1);
  }

  if (istr.eof()) { return rval; }

  int i = istr.peek();
  if (isalpha(i)) {
    std::string label;
    istr >> label;
    bool found = false;
    for (int i=0; units[i].points; ++i) {
      if (units[i].name == label) {
        found = true;
        rval *= units[i].points;
      }
    }
    if (!found) {
      std::cerr << "invalid unit name " << label << "\n";
      exit(1);
    }
  }

  return rval;
  
};

double parse_unit(const std::string& str) {
  std::istringstream istr(str);
  return parse_unit(istr);
}

template <class Tval>
Tval parse_value(const std::string& str) {
  std::istringstream istr(str);
  Tval rval;
  if (!(istr >> rval) || istr.peek() != EOF) {
    std::cerr << "error parsing value\n";
    exit(1);
  } 
  return rval;
}

void usage(std::ostream& ostr) {

  ostr << "usage: maketags FAMILY [OPTIONS]\n\n"
       << "  Known tag families:"; 

  TagFamily::StringArray known = TagFamily::families();
  for (size_t i=0; i<known.size(); ++i) { ostr << " " << known[i]; }
  ostr << "\n\n";

  ostr << "Options:\n\n"
       << "   --paper TYPE               where TYPE is one of: ";
  for (int i=0; papers[i].width_points; ++i) { ostr << papers[i].name << " "; }
  ostr << "\n";
  ostr << "   --paperdims LENGTH LENGTH  dimensions of paper\n"
       << "   --tagsize LENGTH           width of entire tag including white border\n"
       << "   --innersize LENGTH         width of black portion of tag only\n"
       << "   --padding LENGTH           set padding between tags\n"
       << "   --tagfrac N                generate N tags per paper width (set tag size automatically)\n"
       << "   --id N                     generate only tag with id N (mutliple allowed)\n"
       << "   --maxid N                  generate only tags with id's less than N\n"
       << "   --label                    add labels\n"
       << "   --help                     see this message\n"
       << "\n";

 
}

int main(int argc, char** argv) {

  enum SizeType {
    SizeFull,
    SizeInner,
    SizeFrac,
  };

  double width = 612;
  double height = 792;
  SizeType stype = SizeFull;
  double size = width;
  double tagfrac = 1;
  double padding = 0;
  bool draw_labels = false;

  if (argc < 2) {
    usage(std::cerr);
    exit(1);
  }

  std::vector<size_t> ids;

  if (std::string(argv[1]) == "--help") {
    usage(std::cout);
    exit(0);
  }

  TagFamily family(argv[1]);

  for (int i=2; i<argc; ++i) {
    std::string optarg = argv[i];
    if (optarg == "--paper") {
      std::string paper = argv[++i];
      bool found = false;
      for (int j=0; papers[j].width_points; ++j) {
        if (papers[j].name == paper) {
          found = true;
          width = papers[j].width_points;
          height = papers[j].height_points;
        }
      }
      if (!found) {
        std::cerr << "unrecognized paper size: " << paper << "\n";
        exit(1);
      }
    } else if (optarg == "--label") {
      draw_labels = true;
    } else if (optarg == "--tagsize") {
      size = parse_unit(argv[++i]);
      stype = SizeFull;
    } else if (optarg == "--innersize") {
      size = parse_unit(argv[++i]);
      stype = SizeInner;
    } else if (optarg == "--padding") {
      padding = parse_unit(argv[++i]);
    } else if (optarg == "--tagfrac") {
      tagfrac = parse_value<double>(argv[++i]);
      stype = SizeFrac;
      if (tagfrac < 1) {
        std::cerr << "tag fraction must be greater than one \n";
        exit(1);
      }
    } else if (optarg == "--id") {
      ids.push_back(parse_value<size_t>(argv[++i]));
    } else if (optarg == "--maxid") {
      size_t maxid = parse_value<size_t>(argv[++i]);
      for (size_t i=0; i<maxid; ++i) {
        ids.push_back(i);
      }
    } else if (optarg == "--help") {
      usage(std::cout);
      exit(0);
    } else {
      std::cerr << "unrecognized option: " << optarg << "\n";
      exit(1);
    }
  }



  int tags_per_row = 0;
  double sq_size = 0;

  int rd = family.getTagRenderDimension();
  int id = rd - 2*family.whiteBorder;

  if (stype == SizeInner) {
    sq_size = size;
    size = sq_size * double(rd)/id;
  } else if (stype == SizeFrac) {
    size = (width - (tagfrac-1)*padding) / tagfrac;
    tags_per_row = int(tagfrac);
  }

  if (!sq_size) { sq_size = size * double(id/rd); }

  if (!tags_per_row) { 

    if (size > width) {
      std::cerr << "tag size wider than page!\n";
    }
    tags_per_row = 1 + (width - size) / (padding + size);

  }

  double px_size = size / rd;

  double row_size = size;
  double font_size = px_size;

  if (draw_labels) {
    row_size += font_size;
  }

  if (row_size > height) {
    std::cerr << "tag size taller than page!\n";
    exit(1);
  }

  int rows_per_page = 1 + (height - row_size) / (padding + row_size);

  int tags_per_page = tags_per_row * rows_per_page;

  std::vector<size_t> tmp;
  for (size_t i=0; i<ids.size(); ++i) {
    if (ids[i] < family.codes.size()) { tmp.push_back(ids[i]); }
  }
  ids.swap(tmp);

  std::cout << "paper size: " << width << "x" << height << "\n";
  std::cout << "tag size: " << size << "\n";
  std::cout << "padding size: " << padding << "\n";

  if (ids.empty()) {
    std::cout << "all ids\n";
    for (size_t i=0; i<family.codes.size(); ++i) { ids.push_back(i); }
  } else {
    std::cout << "ids: ";
    for (size_t i=0; i<ids.size(); ++i) { std::cout << ids[i] << " "; }
    std::cout << "\n";
  }
  
  int output_pages = int(ceil(double(ids.size()) / tags_per_page));

  std::cout << "tags per row: " << tags_per_row << "\n";
  std::cout << "rows per page: " << rows_per_page << "\n";
  std::cout << "tags per page: " << tags_per_page << "\n";
  std::cout << "output pages: " << output_pages << "\n";

  std::cout << "pixel size: " << px_size << "\n";

  std::cout << "square size: " << sq_size << "\n";

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

  surface = cairo_pdf_surface_create("pdffile.pdf", width, height);
  cr = cairo_create(surface);

  cairo_set_line_width(cr, px_size / 32);

  cairo_select_font_face (cr, "Sans", CAIRO_FONT_SLANT_NORMAL,
                          CAIRO_FONT_WEIGHT_NORMAL);

  cairo_set_font_size (cr, font_size);

  int row = 0;
  int col = 0;
  bool newpage = false;
  int wb = family.whiteBorder;
  int bb = family.blackBorder;
  int tb = wb + bb;


  double mw = 0.5 * (width - size*tags_per_row - padding*(tags_per_row-1));
  double mh = 0.5 * (height - row_size*rows_per_page - padding*(rows_per_page-1));
  
  for (size_t i=0; i<ids.size(); ++i) {

    if (newpage) { 
      cairo_show_page(cr); 
      newpage = false; 
    }

    double x = mw + wb * px_size + col * (size + padding);
    double y = mh + wb * px_size + row * (row_size + padding);
    
    // draw thing at x, y
    cairo_set_source_rgb(cr, 0, 0, 0);
    cairo_new_path(cr);
    cairo_rectangle(cr, x, y, sq_size, sq_size);
    cairo_fill(cr);

    char buf[1024];
    snprintf(buf, 1024, "%u", (unsigned int)ids[i]);

    cairo_text_extents_t extents;
    cairo_text_extents (cr, buf, &extents);

    double tx = 0.5 * (sq_size - extents.width);
    cairo_move_to(cr, x + tx, y + sq_size + px_size * wb + font_size);
    cairo_show_text(cr, buf);

    /*
    cairo_rectangle(cr, x, y + sq_size + px_size * wb, sq_size, font_size);
    cairo_fill(cr);
    */

    cv::Mat_<unsigned char> m = family.makeImage(ids[i]);

    cairo_set_source_rgb(cr, 1, 1, 1);

    for (int i=0; i<family.d; ++i) {
      for (int j=0; j<family.d; ++j) {
        bool w = m(i+tb,j+tb);
        if (w) {
          cairo_new_path(cr);
          cairo_rectangle(cr, x+(j+bb)*px_size, y+(i+bb)*px_size, px_size, px_size);
          //cairo_save(cr);
          cairo_fill_preserve(cr);
          //cairo_restore(cr);
          cairo_stroke(cr);
        }
      }
    }


    ++col;
    if (col >= tags_per_row) {
      col = 0;
      ++row;
      if (row >= rows_per_page) {
        row = 0;
        newpage = true;
      }
    }

  }

  cairo_show_page(cr);
  cairo_surface_destroy(surface);
  cairo_destroy(cr);

  return 0;


}
