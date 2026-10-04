// M06-R11: character-coverage helpers for the embedded 5x7 debug font.
//
// The GUI's glyph() (src/gui.cpp) renders a fixed set of ASCII characters from
// a tiny bitmap font. Historically it only recognized uppercase letters,
// digits, and a few punctuation marks; anything else (notably lowercase
// letters and square brackets) silently fell through to a blank space, so
// diagnostic labels such as "t=...", "[COAST]", "min r ..." rendered with
// missing characters and could not be trusted.
//
// These two inline helpers are the single source of truth for "which
// characters the debug font understands". They are deliberately SDL-free and
// header-only so the headless lander_debug_subsystem_tests can verify the exact
// character coverage of every diagnostic label without building or linking the
// GUI. glyph() in src/gui.cpp delegates to them.
#ifndef LANDER_DEBUG_FONT_HPP
#define LANDER_DEBUG_FONT_HPP

namespace lander {
namespace debug_font {

// Map a character to the one the glyph table is indexed by: ASCII lowercase
// letters become uppercase; everything else passes through unchanged.
inline int normalize(int ch) noexcept {
    if (ch >= 'a' && ch <= 'z') return ch - 'a' + 'A';
    return ch;
}

// True when `ch` maps to a glyph the font can draw (rather than being silently
// dropped). The supported set is: space, A-Z, 0-9, and the punctuation
// - + . : / = % ! ( ) , [ ].
inline bool visible(int ch) noexcept {
    const int n = normalize(ch);
    if (n == ' ') return true;  // an intentional blank cell
    if (n >= 'A' && n <= 'Z') return true;
    if (n >= '0' && n <= '9') return true;
    switch (n) {
        case '-':
        case '+':
        case '.':
        case ':':
        case '/':
        case '=':
        case '%':
        case '!':
        case '(':
        case ')':
        case ',':
        case '[':
        case ']':
            return true;
        default:
            return false;
    }
}

}  // namespace debug_font
}  // namespace lander

#endif  // LANDER_DEBUG_FONT_HPP
