
/* A Bison parser, made by GNU Bison 2.4.1.  */

/* Skeleton implementation for Bison's Yacc-like parsers in C
   
      Copyright (C) 1984, 1989, 1990, 2000, 2001, 2002, 2003, 2004, 2005, 2006
   Free Software Foundation, Inc.
   
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   
   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.  */

/* As a special exception, you may create a larger work that contains
   part or all of the Bison parser skeleton and distribute that work
   under terms of your choice, so long as that work isn't itself a
   parser generator using the skeleton or a modified version thereof
   as a parser skeleton.  Alternatively, if you modify or redistribute
   the parser skeleton itself, you may (at your option) remove this
   special exception, which will cause the skeleton and the resulting
   Bison output files to be licensed under the GNU General Public
   License without this special exception.
   
   This special exception was added by the Free Software Foundation in
   version 2.2 of Bison.  */

/* C LALR(1) parser skeleton written by Richard Stallman, by
   simplifying the original so-called "semantic" parser.  */

/* All symbols defined below should begin with yy or YY, to avoid
   infringing on user name space.  This should be done even for local
   variables, as they might otherwise be expanded by user macros.
   There are some unavoidable exceptions within include files to
   define necessary library symbols; they are noted "INFRINGES ON
   USER NAME SPACE" below.  */

/* Identify Bison output.  */
#define YYBISON 1

/* Bison version.  */
#define YYBISON_VERSION "2.4.1"

/* Skeleton name.  */
#define YYSKELETON_NAME "yacc.c"

/* Pure parsers.  */
#define YYPURE 0

/* Push parsers.  */
#define YYPUSH 0

/* Pull parsers.  */
#define YYPULL 1

/* Using locations.  */
#define YYLSP_NEEDED 0



/* Copy the first part of user declarations.  */

/* Line 189 of yacc.c  */
#line 1 "src/cfgparser/oparse.y"


/*
 * The olsr.org Optimized Link-State Routing daemon(olsrd)
 * Copyright (c) 2004, Andreas Tonnesen(andreto@olsr.org)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions 
 * are met:
 *
 * * Redistributions of source code must retain the above copyright 
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in 
 *   the documentation and/or other materials provided with the 
 *   distribution.
 * * Neither the name of olsr.org, olsrd nor the names of its 
 *   contributors may be used to endorse or promote products derived 
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN 
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Visit http://www.olsr.org for more information.
 *
 * If you find this software useful feel free to make a donation
 * to the project. For more information see the website or contact
 * the copyright holders.
 *
 */


#include "olsrd_conf.h"
#include "../defs.h"
#include "../ipcalc.h"
#include "../net_olsr.h"
#include "../link_set.h"
#include "../olsr.h"

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string.h>

#define PARSER_DEBUG 1

#if PARSER_DEBUG
#define PARSER_DEBUG_PRINTF(x, args...)   printf(x, ##args)
#else
#define PARSER_DEBUG_PRINTF(x, args...)   do { } while (0)
#endif

#define SET_IFS_CONF(ifs, ifcnt, field, value) do { \
	for (; ifcnt>0; ifs=ifs->next, ifcnt--) { \
    ifs->cnfi->field = (value); \
    ifs->cnf->field = (value); \
	} \
} while (0)

#define YYSTYPE struct conf_token *

void yyerror(const char *);
int yylex(void);

static int ifs_in_curr_cfg = 0;

static int add_ipv6_addr(YYSTYPE ipaddr_arg, YYSTYPE prefixlen_arg);

static int lq_mult_helper(YYSTYPE ip_addr_arg, YYSTYPE mult_arg)
{
  union olsr_ip_addr addr;
  int i;
  struct olsr_if *walker;

#if PARSER_DEBUG > 0
  printf("\tLinkQualityMult %s %0.2f\n",
         (ip_addr_arg != NULL) ? ip_addr_arg->string : "any",
         mult_arg->floating);
#endif

  memset(&addr, 0, sizeof(addr));

  if (ip_addr_arg != NULL &&
     inet_pton(olsr_cnf->ip_version, ip_addr_arg->string, &addr) <= 0) {
    fprintf(stderr, "Cannot parse IP address %s.\n", ip_addr_arg->string);
    return -1;
  }

  walker = olsr_cnf->interfaces;

  for (i = 0; i < ifs_in_curr_cfg; i++) {
    struct olsr_lq_mult *mult = malloc(sizeof(*mult));
    if (mult == NULL) {
      fprintf(stderr, "Out of memory (LQ multiplier).\n");
      return -1;
    }

    mult->addr = addr;
    mult->value = (uint32_t)(mult_arg->floating * LINK_LOSS_MULTIPLIER);

    mult->next = walker->cnf->lq_mult;
    walker->cnfi->lq_mult = walker->cnf->lq_mult = mult;
    walker->cnf->orig_lq_mult_cnt++;
    walker->cnfi->orig_lq_mult_cnt=walker->cnf->orig_lq_mult_cnt;

    walker = walker->next;
  }

  if (ip_addr_arg != NULL) {
    free(ip_addr_arg->string);
    free(ip_addr_arg);
  }

  free(mult_arg);

  return 0;
}

static int add_ipv6_addr(YYSTYPE ipaddr_arg, YYSTYPE prefixlen_arg)
{
  union olsr_ip_addr ipaddr;
  PARSER_DEBUG_PRINTF("HNA IPv6 entry: %s/%d\n", ipaddr_arg->string, prefixlen_arg->integer);

  if (olsr_cnf->ip_version != AF_INET6) {
    fprintf(stderr, "IPv6 addresses can only be used if \"IpVersion\" == 6, skipping HNA6.\n");
    olsr_startup_sleep(3);
  }
	else {
	  if(inet_pton(AF_INET6, ipaddr_arg->string, &ipaddr) <= 0) {
      fprintf(stderr, "ihna6entry: Failed converting IP address %s\n", ipaddr_arg->string);
      return 1;
    }

		if (prefixlen_arg->integer > 128) {
			fprintf(stderr, "ihna6entry: Illegal IPv6 prefix length %d\n", prefixlen_arg->integer);
			return 1;
		}

		/* Queue */
		ip_prefix_list_add(&olsr_cnf->hna_entries, &ipaddr, prefixlen_arg->integer);
	}
  free(ipaddr_arg->string);
  free(ipaddr_arg);
  free(prefixlen_arg);

  return 0;
}



/* Line 189 of yacc.c  */
#line 239 "src/cfgparser/oparse.c"

/* Enabling traces.  */
#ifndef YYDEBUG
# define YYDEBUG 0
#endif

/* Enabling verbose error messages.  */
#ifdef YYERROR_VERBOSE
# undef YYERROR_VERBOSE
# define YYERROR_VERBOSE 1
#else
# define YYERROR_VERBOSE 0
#endif

/* Enabling the token table.  */
#ifndef YYTOKEN_TABLE
# define YYTOKEN_TABLE 0
#endif


/* Tokens.  */
#ifndef YYTOKENTYPE
# define YYTOKENTYPE
   /* Put the tokens into the symbol table, so that GDB and other debuggers
      know about them.  */
   enum yytokentype {
     TOK_SLASH = 258,
     TOK_OPEN = 259,
     TOK_CLOSE = 260,
     TOK_STRING = 261,
     TOK_INTEGER = 262,
     TOK_FLOAT = 263,
     TOK_BOOLEAN = 264,
     TOK_IPV6TYPE = 265,
     TOK_DEBUGLEVEL = 266,
     TOK_IPVERSION = 267,
     TOK_HNA4 = 268,
     TOK_HNA6 = 269,
     TOK_PLUGIN = 270,
     TOK_INTERFACE_DEFAULTS = 271,
     TOK_INTERFACE = 272,
     TOK_NOINT = 273,
     TOK_TOS = 274,
     TOK_OLSRPORT = 275,
     TOK_RTPROTO = 276,
     TOK_RTTABLE = 277,
     TOK_RTTABLE_DEFAULT = 278,
     TOK_RTTABLE_TUNNEL = 279,
     TOK_RTTABLE_PRIORITY = 280,
     TOK_RTTABLE_DEFAULTOLSR_PRIORITY = 281,
     TOK_RTTABLE_TUNNEL_PRIORITY = 282,
     TOK_RTTABLE_DEFAULT_PRIORITY = 283,
     TOK_WILLINGNESS = 284,
     TOK_IPCCON = 285,
     TOK_FIBMETRIC = 286,
     TOK_USEHYST = 287,
     TOK_HYSTSCALE = 288,
     TOK_HYSTUPPER = 289,
     TOK_HYSTLOWER = 290,
     TOK_POLLRATE = 291,
     TOK_NICCHGSPOLLRT = 292,
     TOK_TCREDUNDANCY = 293,
     TOK_MPRCOVERAGE = 294,
     TOK_LQ_LEVEL = 295,
     TOK_LQ_FISH = 296,
     TOK_LQ_AGING = 297,
     TOK_LQ_PLUGIN = 298,
     TOK_LQ_NAT_THRESH = 299,
     TOK_LQ_MULT = 300,
     TOK_CLEAR_SCREEN = 301,
     TOK_PLPARAM = 302,
     TOK_MIN_TC_VTIME = 303,
     TOK_LOCK_FILE = 304,
     TOK_USE_NIIT = 305,
     TOK_SMART_GW = 306,
     TOK_SMART_GW_ALLOW_NAT = 307,
     TOK_SMART_GW_UPLINK = 308,
     TOK_SMART_GW_UPLINK_NAT = 309,
     TOK_SMART_GW_SPEED = 310,
     TOK_SMART_GW_PREFIX = 311,
     TOK_SRC_IP_ROUTES = 312,
     TOK_MAIN_IP = 313,
     TOK_HOSTLABEL = 314,
     TOK_NETLABEL = 315,
     TOK_MAXIPC = 316,
     TOK_IFMODE = 317,
     TOK_IPV4BROADCAST = 318,
     TOK_IPV4MULTICAST = 319,
     TOK_IPV6MULTICAST = 320,
     TOK_IPV4SRC = 321,
     TOK_IPV6SRC = 322,
     TOK_IFWEIGHT = 323,
     TOK_HELLOINT = 324,
     TOK_HELLOVAL = 325,
     TOK_TCINT = 326,
     TOK_TCVAL = 327,
     TOK_MIDINT = 328,
     TOK_MIDVAL = 329,
     TOK_HNAINT = 330,
     TOK_HNAVAL = 331,
     TOK_AUTODETCHG = 332,
     TOK_IPV4_ADDR = 333,
     TOK_IPV6_ADDR = 334,
     TOK_DEFAULT = 335,
     TOK_AUTO = 336,
     TOK_NONE = 337,
     TOK_COMMENT = 338
   };
#endif



#if ! defined YYSTYPE && ! defined YYSTYPE_IS_DECLARED
typedef int YYSTYPE;
# define YYSTYPE_IS_TRIVIAL 1
# define yystype YYSTYPE /* obsolescent; will be withdrawn */
# define YYSTYPE_IS_DECLARED 1
#endif


/* Copy the second part of user declarations.  */


/* Line 264 of yacc.c  */
#line 364 "src/cfgparser/oparse.c"

#ifdef short
# undef short
#endif

#ifdef YYTYPE_UINT8
typedef YYTYPE_UINT8 yytype_uint8;
#else
typedef unsigned char yytype_uint8;
#endif

#ifdef YYTYPE_INT8
typedef YYTYPE_INT8 yytype_int8;
#elif (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
typedef signed char yytype_int8;
#else
typedef short int yytype_int8;
#endif

#ifdef YYTYPE_UINT16
typedef YYTYPE_UINT16 yytype_uint16;
#else
typedef unsigned short int yytype_uint16;
#endif

#ifdef YYTYPE_INT16
typedef YYTYPE_INT16 yytype_int16;
#else
typedef short int yytype_int16;
#endif

#ifndef YYSIZE_T
# ifdef __SIZE_TYPE__
#  define YYSIZE_T __SIZE_TYPE__
# elif defined size_t
#  define YYSIZE_T size_t
# elif ! defined YYSIZE_T && (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
#  include <stddef.h> /* INFRINGES ON USER NAME SPACE */
#  define YYSIZE_T size_t
# else
#  define YYSIZE_T unsigned int
# endif
#endif

#define YYSIZE_MAXIMUM ((YYSIZE_T) -1)

#ifndef YY_
# if YYENABLE_NLS
#  if ENABLE_NLS
#   include <libintl.h> /* INFRINGES ON USER NAME SPACE */
#   define YY_(msgid) dgettext ("bison-runtime", msgid)
#  endif
# endif
# ifndef YY_
#  define YY_(msgid) msgid
# endif
#endif

/* Suppress unused-variable warnings by "using" E.  */
#if ! defined lint || defined __GNUC__
# define YYUSE(e) ((void) (e))
#else
# define YYUSE(e) /* empty */
#endif

/* Identity function, used to suppress warnings about constant conditions.  */
#ifndef lint
# define YYID(n) (n)
#else
#if (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
static int
YYID (int yyi)
#else
static int
YYID (yyi)
    int yyi;
#endif
{
  return yyi;
}
#endif

#if ! defined yyoverflow || YYERROR_VERBOSE

/* The parser invokes alloca or malloc; define the necessary symbols.  */

# ifdef YYSTACK_USE_ALLOCA
#  if YYSTACK_USE_ALLOCA
#   ifdef __GNUC__
#    define YYSTACK_ALLOC __builtin_alloca
#   elif defined __BUILTIN_VA_ARG_INCR
#    include <alloca.h> /* INFRINGES ON USER NAME SPACE */
#   elif defined _AIX
#    define YYSTACK_ALLOC __alloca
#   elif defined _MSC_VER
#    include <malloc.h> /* INFRINGES ON USER NAME SPACE */
#    define alloca _alloca
#   else
#    define YYSTACK_ALLOC alloca
#    if ! defined _ALLOCA_H && ! defined _STDLIB_H && (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
#     include <stdlib.h> /* INFRINGES ON USER NAME SPACE */
#     ifndef _STDLIB_H
#      define _STDLIB_H 1
#     endif
#    endif
#   endif
#  endif
# endif

# ifdef YYSTACK_ALLOC
   /* Pacify GCC's `empty if-body' warning.  */
#  define YYSTACK_FREE(Ptr) do { /* empty */; } while (YYID (0))
#  ifndef YYSTACK_ALLOC_MAXIMUM
    /* The OS might guarantee only one guard page at the bottom of the stack,
       and a page size can be as small as 4096 bytes.  So we cannot safely
       invoke alloca (N) if N exceeds 4096.  Use a slightly smaller number
       to allow for a few compiler-allocated temporary stack slots.  */
#   define YYSTACK_ALLOC_MAXIMUM 4032 /* reasonable circa 2006 */
#  endif
# else
#  define YYSTACK_ALLOC YYMALLOC
#  define YYSTACK_FREE YYFREE
#  ifndef YYSTACK_ALLOC_MAXIMUM
#   define YYSTACK_ALLOC_MAXIMUM YYSIZE_MAXIMUM
#  endif
#  if (defined __cplusplus && ! defined _STDLIB_H \
       && ! ((defined YYMALLOC || defined malloc) \
	     && (defined YYFREE || defined free)))
#   include <stdlib.h> /* INFRINGES ON USER NAME SPACE */
#   ifndef _STDLIB_H
#    define _STDLIB_H 1
#   endif
#  endif
#  ifndef YYMALLOC
#   define YYMALLOC malloc
#   if ! defined malloc && ! defined _STDLIB_H && (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
void *malloc (YYSIZE_T); /* INFRINGES ON USER NAME SPACE */
#   endif
#  endif
#  ifndef YYFREE
#   define YYFREE free
#   if ! defined free && ! defined _STDLIB_H && (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
void free (void *); /* INFRINGES ON USER NAME SPACE */
#   endif
#  endif
# endif
#endif /* ! defined yyoverflow || YYERROR_VERBOSE */


#if (! defined yyoverflow \
     && (! defined __cplusplus \
	 || (defined YYSTYPE_IS_TRIVIAL && YYSTYPE_IS_TRIVIAL)))

/* A type that is properly aligned for any stack member.  */
union yyalloc
{
  yytype_int16 yyss_alloc;
  YYSTYPE yyvs_alloc;
};

/* The size of the maximum gap between one aligned stack and the next.  */
# define YYSTACK_GAP_MAXIMUM (sizeof (union yyalloc) - 1)

/* The size of an array large to enough to hold all stacks, each with
   N elements.  */
# define YYSTACK_BYTES(N) \
     ((N) * (sizeof (yytype_int16) + sizeof (YYSTYPE)) \
      + YYSTACK_GAP_MAXIMUM)

/* Copy COUNT objects from FROM to TO.  The source and destination do
   not overlap.  */
# ifndef YYCOPY
#  if defined __GNUC__ && 1 < __GNUC__
#   define YYCOPY(To, From, Count) \
      __builtin_memcpy (To, From, (Count) * sizeof (*(From)))
#  else
#   define YYCOPY(To, From, Count)		\
      do					\
	{					\
	  YYSIZE_T yyi;				\
	  for (yyi = 0; yyi < (Count); yyi++)	\
	    (To)[yyi] = (From)[yyi];		\
	}					\
      while (YYID (0))
#  endif
# endif

/* Relocate STACK from its old location to the new one.  The
   local variables YYSIZE and YYSTACKSIZE give the old and new number of
   elements in the stack, and YYPTR gives the new location of the
   stack.  Advance YYPTR to a properly aligned location for the next
   stack.  */
# define YYSTACK_RELOCATE(Stack_alloc, Stack)				\
    do									\
      {									\
	YYSIZE_T yynewbytes;						\
	YYCOPY (&yyptr->Stack_alloc, Stack, yysize);			\
	Stack = &yyptr->Stack_alloc;					\
	yynewbytes = yystacksize * sizeof (*Stack) + YYSTACK_GAP_MAXIMUM; \
	yyptr += yynewbytes / sizeof (*yyptr);				\
      }									\
    while (YYID (0))

#endif

/* YYFINAL -- State number of the termination state.  */
#define YYFINAL  2
/* YYLAST -- Last index in YYTABLE.  */
#define YYLAST   207

/* YYNTOKENS -- Number of terminals.  */
#define YYNTOKENS  84
/* YYNNTS -- Number of nonterminals.  */
#define YYNNTS  90
/* YYNRULES -- Number of rules.  */
#define YYNRULES  184
/* YYNRULES -- Number of states.  */
#define YYNSTATES  273

/* YYTRANSLATE(YYLEX) -- Bison symbol number corresponding to YYLEX.  */
#define YYUNDEFTOK  2
#define YYMAXUTOK   338

#define YYTRANSLATE(YYX)						\
  ((unsigned int) (YYX) <= YYMAXUTOK ? yytranslate[YYX] : YYUNDEFTOK)

/* YYTRANSLATE[YYLEX] -- Bison symbol number corresponding to YYLEX.  */
static const yytype_uint8 yytranslate[] =
{
       0,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     1,     2,     3,     4,
       5,     6,     7,     8,     9,    10,    11,    12,    13,    14,
      15,    16,    17,    18,    19,    20,    21,    22,    23,    24,
      25,    26,    27,    28,    29,    30,    31,    32,    33,    34,
      35,    36,    37,    38,    39,    40,    41,    42,    43,    44,
      45,    46,    47,    48,    49,    50,    51,    52,    53,    54,
      55,    56,    57,    58,    59,    60,    61,    62,    63,    64,
      65,    66,    67,    68,    69,    70,    71,    72,    73,    74,
      75,    76,    77,    78,    79,    80,    81,    82,    83
};

#if YYDEBUG
/* YYPRHS[YYN] -- Index of the first RHS symbol of rule number YYN in
   YYRHS.  */
static const yytype_uint16 yyprhs[] =
{
       0,     0,     3,     4,     7,    10,    12,    14,    16,    18,
      20,    22,    24,    26,    28,    30,    32,    34,    36,    38,
      40,    42,    44,    46,    48,    50,    52,    54,    56,    58,
      60,    62,    64,    66,    68,    70,    72,    74,    76,    78,
      80,    82,    84,    86,    88,    90,    92,    95,    98,   101,
     104,   107,   110,   114,   115,   118,   120,   122,   126,   127,
     130,   132,   134,   138,   139,   142,   144,   146,   148,   150,
     153,   154,   157,   161,   165,   166,   169,   171,   173,   175,
     177,   179,   181,   183,   185,   187,   189,   191,   193,   195,
     197,   199,   201,   203,   205,   209,   210,   213,   215,   217,
     219,   222,   225,   229,   234,   237,   240,   243,   246,   249,
     252,   255,   258,   261,   264,   267,   270,   273,   276,   279,
     282,   286,   290,   294,   297,   300,   303,   306,   310,   313,
     317,   319,   321,   324,   327,   330,   333,   336,   339,   342,
     345,   348,   351,   354,   357,   360,   363,   366,   369,   372,
     375,   378,   381,   384,   387,   390,   393,   396,   399,   402,
     405,   408,   411,   414,   417,   420,   423,   426,   429,   432,
     435,   438,   441,   444,   447,   450,   454,   457,   461,   466,
     469,   472,   475,   478,   482
};

/* YYRHS -- A `-1'-separated list of the rules' RHS.  */
static const yytype_int16 yyrhs[] =
{
      85,     0,    -1,    -1,    85,    87,    -1,    85,    86,    -1,
     127,    -1,   128,    -1,   129,    -1,   134,    -1,   135,    -1,
     136,    -1,   137,    -1,   138,    -1,   139,    -1,   140,    -1,
     141,    -1,   144,    -1,   143,    -1,   142,    -1,   145,    -1,
     146,    -1,   147,    -1,   148,    -1,   149,    -1,   150,    -1,
     151,    -1,   152,    -1,   153,    -1,   154,    -1,   159,    -1,
     155,    -1,   160,    -1,   156,    -1,   161,    -1,   173,    -1,
     157,    -1,   158,    -1,   162,    -1,   163,    -1,   164,    -1,
     165,    -1,   167,    -1,   166,    -1,   168,    -1,   169,    -1,
     170,    -1,    13,    88,    -1,    14,    91,    -1,    30,    94,
      -1,   106,   100,    -1,    97,    99,    -1,   171,   103,    -1,
       4,    89,     5,    -1,    -1,    89,    90,    -1,   173,    -1,
     130,    -1,     4,    92,     5,    -1,    -1,    92,    93,    -1,
     173,    -1,   131,    -1,     4,    95,     5,    -1,    -1,    95,
      96,    -1,   173,    -1,   107,    -1,   108,    -1,   109,    -1,
     132,    98,    -1,    -1,    98,   133,    -1,     4,   101,     5,
      -1,     4,   101,     5,    -1,    -1,   101,   102,    -1,   173,
      -1,   110,    -1,   111,    -1,   112,    -1,   113,    -1,   114,
      -1,   115,    -1,   116,    -1,   117,    -1,   118,    -1,   119,
      -1,   120,    -1,   121,    -1,   122,    -1,   123,    -1,   124,
      -1,   125,    -1,   126,    -1,     4,   104,     5,    -1,    -1,
     104,   105,    -1,   172,    -1,   173,    -1,    16,    -1,    61,
       7,    -1,    59,    78,    -1,    60,    78,    78,    -1,    60,
      78,     3,     7,    -1,    68,     7,    -1,    62,     6,    -1,
      63,    78,    -1,    64,    78,    -1,    65,    79,    -1,    66,
      78,    -1,    67,    79,    -1,    69,     8,    -1,    70,     8,
      -1,    71,     8,    -1,    72,     8,    -1,    73,     8,    -1,
      74,     8,    -1,    75,     8,    -1,    76,     8,    -1,    77,
       9,    -1,    45,    80,     8,    -1,    45,    78,     8,    -1,
      45,    79,     8,    -1,    11,     7,    -1,    12,     7,    -1,
      31,     6,    -1,    78,    78,    -1,    78,     3,     7,    -1,
      79,     7,    -1,    79,     3,     7,    -1,    17,    -1,     6,
      -1,    18,     9,    -1,    19,     7,    -1,    20,     7,    -1,
      21,     7,    -1,    22,     7,    -1,    22,    81,    -1,    23,
       7,    -1,    23,    81,    -1,    24,     7,    -1,    24,    81,
      -1,    25,     7,    -1,    25,    81,    -1,    25,    82,    -1,
      28,     7,    -1,    28,    81,    -1,    28,    82,    -1,    27,
       7,    -1,    27,    81,    -1,    27,    82,    -1,    26,     7,
      -1,    26,    81,    -1,    26,    82,    -1,    29,     7,    -1,
      32,     9,    -1,    33,     8,    -1,    34,     8,    -1,    35,
       8,    -1,    36,     8,    -1,    37,     8,    -1,    38,     7,
      -1,    39,     7,    -1,    40,     7,    -1,    41,     7,    -1,
      42,     8,    -1,    48,     8,    -1,    49,     6,    -1,    43,
       6,    -1,    44,     8,    -1,    46,     9,    -1,    50,     9,
      -1,    51,     9,    -1,    52,     9,    -1,    53,     6,    -1,
      55,     7,     7,    -1,    54,     9,    -1,    56,    79,     7,
      -1,    56,    79,     3,     7,    -1,    57,     9,    -1,    58,
      78,    -1,    58,    79,    -1,    15,     6,    -1,    47,     6,
       6,    -1,    83,    -1
};

/* YYRLINE[YYN] -- source line where rule number YYN was defined.  */
static const yytype_uint16 yyrline[] =
{
       0,   257,   257,   258,   259,   262,   263,   264,   265,   266,
     267,   268,   269,   270,   271,   272,   273,   274,   275,   276,
     277,   278,   279,   280,   281,   282,   283,   284,   285,   286,
     287,   288,   289,   290,   291,   292,   293,   294,   295,   296,
     297,   298,   299,   300,   301,   302,   305,   306,   307,   308,
     309,   310,   313,   316,   316,   319,   320,   323,   326,   326,
     329,   330,   333,   336,   336,   339,   340,   341,   342,   345,
     348,   348,   351,   354,   368,   368,   371,   372,   373,   374,
     375,   376,   377,   378,   379,   380,   381,   382,   383,   384,
     385,   386,   387,   388,   391,   394,   394,   397,   398,   401,
     432,   439,   456,   479,   503,   524,   539,   559,   579,   599,
     619,   639,   651,   663,   675,   687,   700,   712,   724,   736,
     749,   756,   763,   771,   780,   800,   820,   854,   890,   896,
     904,   911,   967,   975,   984,   992,  1000,  1006,  1014,  1020,
    1028,  1034,  1042,  1048,  1054,  1062,  1068,  1074,  1082,  1088,
    1094,  1102,  1108,  1114,  1122,  1131,  1139,  1147,  1155,  1163,
    1171,  1179,  1187,  1195,  1203,  1211,  1219,  1227,  1234,  1242,
    1250,  1258,  1266,  1274,  1282,  1305,  1315,  1323,  1335,  1349,
    1357,  1368,  1380,  1426,  1449
};
#endif

#if YYDEBUG || YYERROR_VERBOSE || YYTOKEN_TABLE
/* YYTNAME[SYMBOL-NUM] -- String name of the symbol SYMBOL-NUM.
   First, the terminals, then, starting at YYNTOKENS, nonterminals.  */
static const char *const yytname[] =
{
  "$end", "error", "$undefined", "TOK_SLASH", "TOK_OPEN", "TOK_CLOSE",
  "TOK_STRING", "TOK_INTEGER", "TOK_FLOAT", "TOK_BOOLEAN", "TOK_IPV6TYPE",
  "TOK_DEBUGLEVEL", "TOK_IPVERSION", "TOK_HNA4", "TOK_HNA6", "TOK_PLUGIN",
  "TOK_INTERFACE_DEFAULTS", "TOK_INTERFACE", "TOK_NOINT", "TOK_TOS",
  "TOK_OLSRPORT", "TOK_RTPROTO", "TOK_RTTABLE", "TOK_RTTABLE_DEFAULT",
  "TOK_RTTABLE_TUNNEL", "TOK_RTTABLE_PRIORITY",
  "TOK_RTTABLE_DEFAULTOLSR_PRIORITY", "TOK_RTTABLE_TUNNEL_PRIORITY",
  "TOK_RTTABLE_DEFAULT_PRIORITY", "TOK_WILLINGNESS", "TOK_IPCCON",
  "TOK_FIBMETRIC", "TOK_USEHYST", "TOK_HYSTSCALE", "TOK_HYSTUPPER",
  "TOK_HYSTLOWER", "TOK_POLLRATE", "TOK_NICCHGSPOLLRT", "TOK_TCREDUNDANCY",
  "TOK_MPRCOVERAGE", "TOK_LQ_LEVEL", "TOK_LQ_FISH", "TOK_LQ_AGING",
  "TOK_LQ_PLUGIN", "TOK_LQ_NAT_THRESH", "TOK_LQ_MULT", "TOK_CLEAR_SCREEN",
  "TOK_PLPARAM", "TOK_MIN_TC_VTIME", "TOK_LOCK_FILE", "TOK_USE_NIIT",
  "TOK_SMART_GW", "TOK_SMART_GW_ALLOW_NAT", "TOK_SMART_GW_UPLINK",
  "TOK_SMART_GW_UPLINK_NAT", "TOK_SMART_GW_SPEED", "TOK_SMART_GW_PREFIX",
  "TOK_SRC_IP_ROUTES", "TOK_MAIN_IP", "TOK_HOSTLABEL", "TOK_NETLABEL",
  "TOK_MAXIPC", "TOK_IFMODE", "TOK_IPV4BROADCAST", "TOK_IPV4MULTICAST",
  "TOK_IPV6MULTICAST", "TOK_IPV4SRC", "TOK_IPV6SRC", "TOK_IFWEIGHT",
  "TOK_HELLOINT", "TOK_HELLOVAL", "TOK_TCINT", "TOK_TCVAL", "TOK_MIDINT",
  "TOK_MIDVAL", "TOK_HNAINT", "TOK_HNAVAL", "TOK_AUTODETCHG",
  "TOK_IPV4_ADDR", "TOK_IPV6_ADDR", "TOK_DEFAULT", "TOK_AUTO", "TOK_NONE",
  "TOK_COMMENT", "$accept", "conf", "stmt", "block", "hna4body",
  "hna4stmts", "hna4stmt", "hna6body", "hna6stmts", "hna6stmt", "ipcbody",
  "ipcstmts", "ipcstmt", "ifblock", "ifnicks", "ifbody", "ifdbody",
  "ifstmts", "ifstmt", "plbody", "plstmts", "plstmt", "ifdblock",
  "imaxipc", "ipchost", "ipcnet", "iifweight", "isetifmode", "isetipv4br",
  "isetipv4mc", "isetipv6mc", "isetipv4src", "isetipv6src", "isethelloint",
  "isethelloval", "isettcint", "isettcval", "isetmidint", "isetmidval",
  "isethnaint", "isethnaval", "isetautodetchg", "isetlqmult", "idebug",
  "iipversion", "fibmetric", "ihna4entry", "ihna6entry", "ifstart",
  "ifnick", "bnoint", "atos", "aolsrport", "irtproto", "irttable",
  "irttable_default", "irttable_tunnel", "irttable_priority",
  "irttable_default_priority", "irttable_tunnel_priority",
  "irttable_defaultolsr_priority", "awillingness", "busehyst",
  "fhystscale", "fhystupper", "fhystlower", "fpollrate", "fnicchgspollrt",
  "atcredundancy", "amprcoverage", "alq_level", "alq_fish", "alq_aging",
  "amin_tc_vtime", "alock_file", "alq_plugin", "anat_thresh",
  "bclear_screen", "suse_niit", "bsmart_gw", "bsmart_gw_allow_nat",
  "ssmart_gw_uplink", "ismart_gw_speed", "bsmart_gw_uplink_nat",
  "ismart_gw_prefix", "bsrc_ip_routes", "amain_ip", "plblock", "plparam",
  "vcomment", 0
};
#endif

# ifdef YYPRINT
/* YYTOKNUM[YYLEX-NUM] -- Internal token number corresponding to
   token YYLEX-NUM.  */
static const yytype_uint16 yytoknum[] =
{
       0,   256,   257,   258,   259,   260,   261,   262,   263,   264,
     265,   266,   267,   268,   269,   270,   271,   272,   273,   274,
     275,   276,   277,   278,   279,   280,   281,   282,   283,   284,
     285,   286,   287,   288,   289,   290,   291,   292,   293,   294,
     295,   296,   297,   298,   299,   300,   301,   302,   303,   304,
     305,   306,   307,   308,   309,   310,   311,   312,   313,   314,
     315,   316,   317,   318,   319,   320,   321,   322,   323,   324,
     325,   326,   327,   328,   329,   330,   331,   332,   333,   334,
     335,   336,   337,   338
};
# endif

/* YYR1[YYN] -- Symbol number of symbol that rule YYN derives.  */
static const yytype_uint8 yyr1[] =
{
       0,    84,    85,    85,    85,    86,    86,    86,    86,    86,
      86,    86,    86,    86,    86,    86,    86,    86,    86,    86,
      86,    86,    86,    86,    86,    86,    86,    86,    86,    86,
      86,    86,    86,    86,    86,    86,    86,    86,    86,    86,
      86,    86,    86,    86,    86,    86,    87,    87,    87,    87,
      87,    87,    88,    89,    89,    90,    90,    91,    92,    92,
      93,    93,    94,    95,    95,    96,    96,    96,    96,    97,
      98,    98,    99,   100,   101,   101,   102,   102,   102,   102,
     102,   102,   102,   102,   102,   102,   102,   102,   102,   102,
     102,   102,   102,   102,   103,   104,   104,   105,   105,   106,
     107,   108,   109,   109,   110,   111,   112,   113,   114,   115,
     116,   117,   118,   119,   120,   121,   122,   123,   124,   125,
     126,   126,   126,   127,   128,   129,   130,   130,   131,   131,
     132,   133,   134,   135,   136,   137,   138,   138,   139,   139,
     140,   140,   141,   141,   141,   142,   142,   142,   143,   143,
     143,   144,   144,   144,   145,   146,   147,   148,   149,   150,
     151,   152,   153,   154,   155,   156,   157,   158,   159,   160,
     161,   162,   163,   164,   165,   166,   167,   168,   168,   169,
     170,   170,   171,   172,   173
};

/* YYR2[YYN] -- Number of symbols composing right hand side of rule YYN.  */
static const yytype_uint8 yyr2[] =
{
       0,     2,     0,     2,     2,     1,     1,     1,     1,     1,
       1,     1,     1,     1,     1,     1,     1,     1,     1,     1,
       1,     1,     1,     1,     1,     1,     1,     1,     1,     1,
       1,     1,     1,     1,     1,     1,     1,     1,     1,     1,
       1,     1,     1,     1,     1,     1,     2,     2,     2,     2,
       2,     2,     3,     0,     2,     1,     1,     3,     0,     2,
       1,     1,     3,     0,     2,     1,     1,     1,     1,     2,
       0,     2,     3,     3,     0,     2,     1,     1,     1,     1,
       1,     1,     1,     1,     1,     1,     1,     1,     1,     1,
       1,     1,     1,     1,     3,     0,     2,     1,     1,     1,
       2,     2,     3,     4,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       3,     3,     3,     2,     2,     2,     2,     3,     2,     3,
       1,     1,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     3,     2,     3,     4,     2,
       2,     2,     2,     3,     1
};

/* YYDEFACT[STATE-NAME] -- Default rule to reduce with in state
   STATE-NUM when YYTABLE doesn't specify something else to do.  Zero
   means the default is an error.  */
static const yytype_uint8 yydefact[] =
{
       2,     0,     1,     0,     0,     0,     0,     0,    99,   130,
       0,     0,     0,     0,     0,     0,     0,     0,     0,     0,
       0,     0,     0,     0,     0,     0,     0,     0,     0,     0,
       0,     0,     0,     0,     0,     0,     0,     0,     0,     0,
       0,     0,     0,     0,     0,     0,     0,     0,     0,   184,
       4,     3,     0,     0,     5,     6,     7,    70,     8,     9,
      10,    11,    12,    13,    14,    15,    18,    17,    16,    19,
      20,    21,    22,    23,    24,    25,    26,    27,    28,    30,
      32,    35,    36,    29,    31,    33,    37,    38,    39,    40,
      42,    41,    43,    44,    45,     0,    34,   123,   124,    53,
      46,    58,    47,   182,   132,   133,   134,   135,   136,   137,
     138,   139,   140,   141,   142,   143,   144,   151,   152,   153,
     148,   149,   150,   145,   146,   147,   154,    63,    48,   125,
     155,   156,   157,   158,   159,   160,   161,   162,   163,   164,
     165,   168,   169,   170,   166,   167,   171,   172,   173,   174,
     176,     0,     0,   179,   180,   181,    74,    50,    74,    49,
      69,    95,    51,     0,     0,     0,   175,     0,   177,     0,
       0,   131,    71,     0,    52,     0,    54,    56,    55,    57,
       0,    59,    61,    60,    62,     0,     0,     0,    64,    66,
      67,    68,    65,   178,    72,     0,     0,     0,     0,     0,
       0,     0,     0,     0,     0,     0,     0,     0,     0,     0,
       0,     0,    75,    77,    78,    79,    80,    81,    82,    83,
      84,    85,    86,    87,    88,    89,    90,    91,    92,    93,
      76,    73,    94,     0,    96,    97,    98,     0,   126,     0,
     128,   101,     0,   100,     0,     0,     0,   105,   106,   107,
     108,   109,   110,   104,   111,   112,   113,   114,   115,   116,
     117,   118,   119,     0,   127,   129,     0,   102,   121,   122,
     120,   183,   103
};

/* YYDEFGOTO[NTERM-NUM].  */
static const yytype_int16 yydefgoto[] =
{
      -1,     1,    50,    51,   100,   163,   176,   102,   164,   181,
     128,   165,   188,    52,   160,   157,   159,   169,   212,   162,
     173,   234,    53,   189,   190,   191,   213,   214,   215,   216,
     217,   218,   219,   220,   221,   222,   223,   224,   225,   226,
     227,   228,   229,    54,    55,    56,   177,   182,    57,   172,
      58,    59,    60,    61,    62,    63,    64,    65,    66,    67,
      68,    69,    70,    71,    72,    73,    74,    75,    76,    77,
      78,    79,    80,    81,    82,    83,    84,    85,    86,    87,
      88,    89,    90,    91,    92,    93,    94,    95,   235,   230
};

/* YYPACT[STATE-NUM] -- Index in YYTABLE of the portion describing
   STATE-NUM.  */
#define YYPACT_NINF -77
static const yytype_int16 yypact[] =
{
     -77,     1,   -77,    39,    82,     6,     7,    91,   -77,   -77,
      89,    97,    98,   100,    94,    95,    96,    84,    86,    88,
      92,   101,   105,   104,   102,   106,   107,   108,   125,   126,
     128,   129,   130,   132,   133,   134,   135,   103,   136,   139,
     137,   138,   140,   142,   141,   144,    34,   143,   -73,   -77,
     -77,   -77,   149,   150,   -77,   -77,   -77,   -77,   -77,   -77,
     -77,   -77,   -77,   -77,   -77,   -77,   -77,   -77,   -77,   -77,
     -77,   -77,   -77,   -77,   -77,   -77,   -77,   -77,   -77,   -77,
     -77,   -77,   -77,   -77,   -77,   -77,   -77,   -77,   -77,   -77,
     -77,   -77,   -77,   -77,   -77,   152,   -77,   -77,   -77,   -77,
     -77,   -77,   -77,   -77,   -77,   -77,   -77,   -77,   -77,   -77,
     -77,   -77,   -77,   -77,   -77,   -77,   -77,   -77,   -77,   -77,
     -77,   -77,   -77,   -77,   -77,   -77,   -77,   -77,   -77,   -77,
     -77,   -77,   -77,   -77,   -77,   -77,   -77,   -77,   -77,   -77,
     -77,   -77,   -77,   -77,   -77,   -77,   -77,   -77,   -77,   -77,
     -77,   153,    85,   -77,   -77,   -77,   -77,   -77,   -77,   -77,
     151,   -77,   -77,     4,    76,     2,   -77,   154,   -77,     3,
      55,   -77,   -77,    59,   -77,    80,   -77,   -77,   -77,   -77,
      87,   -77,   -77,   -77,   -77,   109,   110,   171,   -77,   -77,
     -77,   -77,   -77,   -77,   -77,   -76,   173,   111,   112,   113,
     115,   116,   174,   172,   175,   176,   177,   178,   183,   186,
     188,   189,   -77,   -77,   -77,   -77,   -77,   -77,   -77,   -77,
     -77,   -77,   -77,   -77,   -77,   -77,   -77,   -77,   -77,   -77,
     -77,   -77,   -77,   191,   -77,   -77,   -77,   192,   -77,   193,
     -77,   -77,    93,   -77,   194,   195,   196,   -77,   -77,   -77,
     -77,   -77,   -77,   -77,   -77,   -77,   -77,   -77,   -77,   -77,
     -77,   -77,   -77,   199,   -77,   -77,   200,   -77,   -77,   -77,
     -77,   -77,   -77
};

/* YYPGOTO[NTERM-NUM].  */
static const yytype_int8 yypgoto[] =
{
     -77,   -77,   -77,   -77,   -77,   -77,   -77,   -77,   -77,   -77,
     -77,   -77,   -77,   -77,   -77,   -77,   -77,    24,   -77,   -77,
     -77,   -77,   -77,   -77,   -77,   -77,   -77,   -77,   -77,   -77,
     -77,   -77,   -77,   -77,   -77,   -77,   -77,   -77,   -77,   -77,
     -77,   -77,   -77,   -77,   -77,   -77,   -77,   -77,   -77,   -77,
     -77,   -77,   -77,   -77,   -77,   -77,   -77,   -77,   -77,   -77,
     -77,   -77,   -77,   -77,   -77,   -77,   -77,   -77,   -77,   -77,
     -77,   -77,   -77,   -77,   -77,   -77,   -77,   -77,   -77,   -77,
     -77,   -77,   -77,   -77,   -77,   -77,   -77,   -77,   -77,    -1
};

/* YYTABLE[YYPACT[STATE-NUM]].  What to do in state STATE-NUM.  If
   positive, shift that token.  If negative, reduce the rule which
   number is the opposite.  If zero, do what YYDEFACT says.
   If YYTABLE_NINF, syntax error.  */
#define YYTABLE_NINF -1
static const yytype_uint16 yytable[] =
{
      96,     2,   244,   245,   246,   154,   155,   184,   194,   174,
      99,   101,     3,     4,     5,     6,     7,     8,     9,    10,
      11,    12,    13,    14,    15,    16,    17,    18,    19,    20,
      21,    22,    23,    24,    25,    26,    27,    28,    29,    30,
      31,    32,    33,    34,    35,    36,    97,    37,   195,    38,
      39,    40,    41,    42,    43,    44,    45,    46,    47,    48,
     231,   185,   186,   187,   232,   196,   197,   198,   199,   200,
     201,   202,   203,   204,   205,   206,   207,   208,   209,   210,
     211,   179,   175,   237,    49,    49,    49,    49,   167,    98,
     239,   114,   168,   117,   240,   120,   266,   103,   104,   123,
     195,   108,   110,   112,   105,   106,   233,   107,   126,   127,
     129,   130,   143,   152,   131,   132,   133,   196,   197,   198,
     199,   200,   201,   202,   203,   204,   205,   206,   207,   208,
     209,   210,   211,   134,   135,   136,   137,   138,    49,   139,
     141,   140,    49,   142,   144,   145,   146,   147,   149,   148,
     150,   151,   153,   156,   158,   180,   161,   171,   238,    49,
     166,   193,   178,   183,   192,   115,   116,   118,   119,   121,
     122,   267,   236,   124,   125,   109,   111,   113,   243,   247,
     254,   253,   170,   255,   256,   257,   258,   241,   242,   248,
     249,   259,   250,   251,   260,   252,   261,   263,   262,   264,
     265,     0,   268,   269,   270,   271,     0,   272
};

static const yytype_int16 yycheck[] =
{
       1,     0,    78,    79,    80,    78,    79,     5,     5,     5,
       4,     4,    11,    12,    13,    14,    15,    16,    17,    18,
      19,    20,    21,    22,    23,    24,    25,    26,    27,    28,
      29,    30,    31,    32,    33,    34,    35,    36,    37,    38,
      39,    40,    41,    42,    43,    44,     7,    46,    45,    48,
      49,    50,    51,    52,    53,    54,    55,    56,    57,    58,
       5,    59,    60,    61,     5,    62,    63,    64,    65,    66,
      67,    68,    69,    70,    71,    72,    73,    74,    75,    76,
      77,     5,    78,     3,    83,    83,    83,    83,     3,     7,
       3,     7,     7,     7,     7,     7,     3,     6,     9,     7,
      45,     7,     7,     7,     7,     7,    47,     7,     7,     4,
       6,     9,     9,    79,     8,     8,     8,    62,    63,    64,
      65,    66,    67,    68,    69,    70,    71,    72,    73,    74,
      75,    76,    77,     8,     8,     7,     7,     7,    83,     7,
       6,     8,    83,     8,     8,     6,     9,     9,     6,     9,
       9,     7,     9,     4,     4,    79,     4,     6,    78,    83,
       7,     7,   163,   164,   165,    81,    82,    81,    82,    81,
      82,    78,   173,    81,    82,    81,    81,    81,     7,     6,
       8,     7,   158,     8,     8,     8,     8,    78,    78,    78,
      78,     8,    79,    78,     8,    79,     8,     6,     9,     7,
       7,    -1,     8,     8,     8,     6,    -1,     7
};

/* YYSTOS[STATE-NUM] -- The (internal number of the) accessing
   symbol of state STATE-NUM.  */
static const yytype_uint8 yystos[] =
{
       0,    85,     0,    11,    12,    13,    14,    15,    16,    17,
      18,    19,    20,    21,    22,    23,    24,    25,    26,    27,
      28,    29,    30,    31,    32,    33,    34,    35,    36,    37,
      38,    39,    40,    41,    42,    43,    44,    46,    48,    49,
      50,    51,    52,    53,    54,    55,    56,    57,    58,    83,
      86,    87,    97,   106,   127,   128,   129,   132,   134,   135,
     136,   137,   138,   139,   140,   141,   142,   143,   144,   145,
     146,   147,   148,   149,   150,   151,   152,   153,   154,   155,
     156,   157,   158,   159,   160,   161,   162,   163,   164,   165,
     166,   167,   168,   169,   170,   171,   173,     7,     7,     4,
      88,     4,    91,     6,     9,     7,     7,     7,     7,    81,
       7,    81,     7,    81,     7,    81,    82,     7,    81,    82,
       7,    81,    82,     7,    81,    82,     7,     4,    94,     6,
       9,     8,     8,     8,     8,     8,     7,     7,     7,     7,
       8,     6,     8,     9,     8,     6,     9,     9,     9,     6,
       9,     7,    79,     9,    78,    79,     4,    99,     4,   100,
      98,     4,   103,    89,    92,    95,     7,     3,     7,   101,
     101,     6,   133,   104,     5,    78,    90,   130,   173,     5,
      79,    93,   131,   173,     5,    59,    60,    61,    96,   107,
     108,   109,   173,     7,     5,    45,    62,    63,    64,    65,
      66,    67,    68,    69,    70,    71,    72,    73,    74,    75,
      76,    77,   102,   110,   111,   112,   113,   114,   115,   116,
     117,   118,   119,   120,   121,   122,   123,   124,   125,   126,
     173,     5,     5,    47,   105,   172,   173,     3,    78,     3,
       7,    78,    78,     7,    78,    79,    80,     6,    78,    78,
      79,    78,    79,     7,     8,     8,     8,     8,     8,     8,
       8,     8,     9,     6,     7,     7,     3,    78,     8,     8,
       8,     6,     7
};

#define yyerrok		(yyerrstatus = 0)
#define yyclearin	(yychar = YYEMPTY)
#define YYEMPTY		(-2)
#define YYEOF		0

#define YYACCEPT	goto yyacceptlab
#define YYABORT		goto yyabortlab
#define YYERROR		goto yyerrorlab


/* Like YYERROR except do call yyerror.  This remains here temporarily
   to ease the transition to the new meaning of YYERROR, for GCC.
   Once GCC version 2 has supplanted version 1, this can go.  */

#define YYFAIL		goto yyerrlab

#define YYRECOVERING()  (!!yyerrstatus)

#define YYBACKUP(Token, Value)					\
do								\
  if (yychar == YYEMPTY && yylen == 1)				\
    {								\
      yychar = (Token);						\
      yylval = (Value);						\
      yytoken = YYTRANSLATE (yychar);				\
      YYPOPSTACK (1);						\
      goto yybackup;						\
    }								\
  else								\
    {								\
      yyerror (YY_("syntax error: cannot back up")); \
      YYERROR;							\
    }								\
while (YYID (0))


#define YYTERROR	1
#define YYERRCODE	256


/* YYLLOC_DEFAULT -- Set CURRENT to span from RHS[1] to RHS[N].
   If N is 0, then set CURRENT to the empty location which ends
   the previous symbol: RHS[0] (always defined).  */

#define YYRHSLOC(Rhs, K) ((Rhs)[K])
#ifndef YYLLOC_DEFAULT
# define YYLLOC_DEFAULT(Current, Rhs, N)				\
    do									\
      if (YYID (N))                                                    \
	{								\
	  (Current).first_line   = YYRHSLOC (Rhs, 1).first_line;	\
	  (Current).first_column = YYRHSLOC (Rhs, 1).first_column;	\
	  (Current).last_line    = YYRHSLOC (Rhs, N).last_line;		\
	  (Current).last_column  = YYRHSLOC (Rhs, N).last_column;	\
	}								\
      else								\
	{								\
	  (Current).first_line   = (Current).last_line   =		\
	    YYRHSLOC (Rhs, 0).last_line;				\
	  (Current).first_column = (Current).last_column =		\
	    YYRHSLOC (Rhs, 0).last_column;				\
	}								\
    while (YYID (0))
#endif


/* YY_LOCATION_PRINT -- Print the location on the stream.
   This macro was not mandated originally: define only if we know
   we won't break user code: when these are the locations we know.  */

#ifndef YY_LOCATION_PRINT
# if YYLTYPE_IS_TRIVIAL
#  define YY_LOCATION_PRINT(File, Loc)			\
     fprintf (File, "%d.%d-%d.%d",			\
	      (Loc).first_line, (Loc).first_column,	\
	      (Loc).last_line,  (Loc).last_column)
# else
#  define YY_LOCATION_PRINT(File, Loc) ((void) 0)
# endif
#endif


/* YYLEX -- calling `yylex' with the right arguments.  */

#ifdef YYLEX_PARAM
# define YYLEX yylex (YYLEX_PARAM)
#else
# define YYLEX yylex ()
#endif

/* Enable debugging if requested.  */
#if YYDEBUG

# ifndef YYFPRINTF
#  include <stdio.h> /* INFRINGES ON USER NAME SPACE */
#  define YYFPRINTF fprintf
# endif

# define YYDPRINTF(Args)			\
do {						\
  if (yydebug)					\
    YYFPRINTF Args;				\
} while (YYID (0))

# define YY_SYMBOL_PRINT(Title, Type, Value, Location)			  \
do {									  \
  if (yydebug)								  \
    {									  \
      YYFPRINTF (stderr, "%s ", Title);					  \
      yy_symbol_print (stderr,						  \
		  Type, Value); \
      YYFPRINTF (stderr, "\n");						  \
    }									  \
} while (YYID (0))


/*--------------------------------.
| Print this symbol on YYOUTPUT.  |
`--------------------------------*/

/*ARGSUSED*/
#if (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
static void
yy_symbol_value_print (FILE *yyoutput, int yytype, YYSTYPE const * const yyvaluep)
#else
static void
yy_symbol_value_print (yyoutput, yytype, yyvaluep)
    FILE *yyoutput;
    int yytype;
    YYSTYPE const * const yyvaluep;
#endif
{
  if (!yyvaluep)
    return;
# ifdef YYPRINT
  if (yytype < YYNTOKENS)
    YYPRINT (yyoutput, yytoknum[yytype], *yyvaluep);
# else
  YYUSE (yyoutput);
# endif
  switch (yytype)
    {
      default:
	break;
    }
}


/*--------------------------------.
| Print this symbol on YYOUTPUT.  |
`--------------------------------*/

#if (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
static void
yy_symbol_print (FILE *yyoutput, int yytype, YYSTYPE const * const yyvaluep)
#else
static void
yy_symbol_print (yyoutput, yytype, yyvaluep)
    FILE *yyoutput;
    int yytype;
    YYSTYPE const * const yyvaluep;
#endif
{
  if (yytype < YYNTOKENS)
    YYFPRINTF (yyoutput, "token %s (", yytname[yytype]);
  else
    YYFPRINTF (yyoutput, "nterm %s (", yytname[yytype]);

  yy_symbol_value_print (yyoutput, yytype, yyvaluep);
  YYFPRINTF (yyoutput, ")");
}

/*------------------------------------------------------------------.
| yy_stack_print -- Print the state stack from its BOTTOM up to its |
| TOP (included).                                                   |
`------------------------------------------------------------------*/

#if (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
static void
yy_stack_print (yytype_int16 *yybottom, yytype_int16 *yytop)
#else
static void
yy_stack_print (yybottom, yytop)
    yytype_int16 *yybottom;
    yytype_int16 *yytop;
#endif
{
  YYFPRINTF (stderr, "Stack now");
  for (; yybottom <= yytop; yybottom++)
    {
      int yybot = *yybottom;
      YYFPRINTF (stderr, " %d", yybot);
    }
  YYFPRINTF (stderr, "\n");
}

# define YY_STACK_PRINT(Bottom, Top)				\
do {								\
  if (yydebug)							\
    yy_stack_print ((Bottom), (Top));				\
} while (YYID (0))


/*------------------------------------------------.
| Report that the YYRULE is going to be reduced.  |
`------------------------------------------------*/

#if (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
static void
yy_reduce_print (YYSTYPE *yyvsp, int yyrule)
#else
static void
yy_reduce_print (yyvsp, yyrule)
    YYSTYPE *yyvsp;
    int yyrule;
#endif
{
  int yynrhs = yyr2[yyrule];
  int yyi;
  unsigned long int yylno = yyrline[yyrule];
  YYFPRINTF (stderr, "Reducing stack by rule %d (line %lu):\n",
	     yyrule - 1, yylno);
  /* The symbols being reduced.  */
  for (yyi = 0; yyi < yynrhs; yyi++)
    {
      YYFPRINTF (stderr, "   $%d = ", yyi + 1);
      yy_symbol_print (stderr, yyrhs[yyprhs[yyrule] + yyi],
		       &(yyvsp[(yyi + 1) - (yynrhs)])
		       		       );
      YYFPRINTF (stderr, "\n");
    }
}

# define YY_REDUCE_PRINT(Rule)		\
do {					\
  if (yydebug)				\
    yy_reduce_print (yyvsp, Rule); \
} while (YYID (0))

/* Nonzero means print parse trace.  It is left uninitialized so that
   multiple parsers can coexist.  */
int yydebug;
#else /* !YYDEBUG */
# define YYDPRINTF(Args)
# define YY_SYMBOL_PRINT(Title, Type, Value, Location)
# define YY_STACK_PRINT(Bottom, Top)
# define YY_REDUCE_PRINT(Rule)
#endif /* !YYDEBUG */


/* YYINITDEPTH -- initial size of the parser's stacks.  */
#ifndef	YYINITDEPTH
# define YYINITDEPTH 200
#endif

/* YYMAXDEPTH -- maximum size the stacks can grow to (effective only
   if the built-in stack extension method is used).

   Do not make this value too large; the results are undefined if
   YYSTACK_ALLOC_MAXIMUM < YYSTACK_BYTES (YYMAXDEPTH)
   evaluated with infinite-precision integer arithmetic.  */

#ifndef YYMAXDEPTH
# define YYMAXDEPTH 10000
#endif



#if YYERROR_VERBOSE

# ifndef yystrlen
#  if defined __GLIBC__ && defined _STRING_H
#   define yystrlen strlen
#  else
/* Return the length of YYSTR.  */
#if (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
static YYSIZE_T
yystrlen (const char *yystr)
#else
static YYSIZE_T
yystrlen (yystr)
    const char *yystr;
#endif
{
  YYSIZE_T yylen;
  for (yylen = 0; yystr[yylen]; yylen++)
    continue;
  return yylen;
}
#  endif
# endif

# ifndef yystpcpy
#  if defined __GLIBC__ && defined _STRING_H && defined _GNU_SOURCE
#   define yystpcpy stpcpy
#  else
/* Copy YYSRC to YYDEST, returning the address of the terminating '\0' in
   YYDEST.  */
#if (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
static char *
yystpcpy (char *yydest, const char *yysrc)
#else
static char *
yystpcpy (yydest, yysrc)
    char *yydest;
    const char *yysrc;
#endif
{
  char *yyd = yydest;
  const char *yys = yysrc;

  while ((*yyd++ = *yys++) != '\0')
    continue;

  return yyd - 1;
}
#  endif
# endif

# ifndef yytnamerr
/* Copy to YYRES the contents of YYSTR after stripping away unnecessary
   quotes and backslashes, so that it's suitable for yyerror.  The
   heuristic is that double-quoting is unnecessary unless the string
   contains an apostrophe, a comma, or backslash (other than
   backslash-backslash).  YYSTR is taken from yytname.  If YYRES is
   null, do not copy; instead, return the length of what the result
   would have been.  */
static YYSIZE_T
yytnamerr (char *yyres, const char *yystr)
{
  if (*yystr == '"')
    {
      YYSIZE_T yyn = 0;
      char const *yyp = yystr;

      for (;;)
	switch (*++yyp)
	  {
	  case '\'':
	  case ',':
	    goto do_not_strip_quotes;

	  case '\\':
	    if (*++yyp != '\\')
	      goto do_not_strip_quotes;
	    /* Fall through.  */
	  default:
	    if (yyres)
	      yyres[yyn] = *yyp;
	    yyn++;
	    break;

	  case '"':
	    if (yyres)
	      yyres[yyn] = '\0';
	    return yyn;
	  }
    do_not_strip_quotes: ;
    }

  if (! yyres)
    return yystrlen (yystr);

  return yystpcpy (yyres, yystr) - yyres;
}
# endif

/* Copy into YYRESULT an error message about the unexpected token
   YYCHAR while in state YYSTATE.  Return the number of bytes copied,
   including the terminating null byte.  If YYRESULT is null, do not
   copy anything; just return the number of bytes that would be
   copied.  As a special case, return 0 if an ordinary "syntax error"
   message will do.  Return YYSIZE_MAXIMUM if overflow occurs during
   size calculation.  */
static YYSIZE_T
yysyntax_error (char *yyresult, int yystate, int yychar)
{
  int yyn = yypact[yystate];

  if (! (YYPACT_NINF < yyn && yyn <= YYLAST))
    return 0;
  else
    {
      int yytype = YYTRANSLATE (yychar);
      YYSIZE_T yysize0 = yytnamerr (0, yytname[yytype]);
      YYSIZE_T yysize = yysize0;
      YYSIZE_T yysize1;
      int yysize_overflow = 0;
      enum { YYERROR_VERBOSE_ARGS_MAXIMUM = 5 };
      char const *yyarg[YYERROR_VERBOSE_ARGS_MAXIMUM];
      int yyx;

# if 0
      /* This is so xgettext sees the translatable formats that are
	 constructed on the fly.  */
      YY_("syntax error, unexpected %s");
      YY_("syntax error, unexpected %s, expecting %s");
      YY_("syntax error, unexpected %s, expecting %s or %s");
      YY_("syntax error, unexpected %s, expecting %s or %s or %s");
      YY_("syntax error, unexpected %s, expecting %s or %s or %s or %s");
# endif
      char *yyfmt;
      char const *yyf;
      static char const yyunexpected[] = "syntax error, unexpected %s";
      static char const yyexpecting[] = ", expecting %s";
      static char const yyor[] = " or %s";
      char yyformat[sizeof yyunexpected
		    + sizeof yyexpecting - 1
		    + ((YYERROR_VERBOSE_ARGS_MAXIMUM - 2)
		       * (sizeof yyor - 1))];
      char const *yyprefix = yyexpecting;

      /* Start YYX at -YYN if negative to avoid negative indexes in
	 YYCHECK.  */
      int yyxbegin = yyn < 0 ? -yyn : 0;

      /* Stay within bounds of both yycheck and yytname.  */
      int yychecklim = YYLAST - yyn + 1;
      int yyxend = yychecklim < YYNTOKENS ? yychecklim : YYNTOKENS;
      int yycount = 1;

      yyarg[0] = yytname[yytype];
      yyfmt = yystpcpy (yyformat, yyunexpected);

      for (yyx = yyxbegin; yyx < yyxend; ++yyx)
	if (yycheck[yyx + yyn] == yyx && yyx != YYTERROR)
	  {
	    if (yycount == YYERROR_VERBOSE_ARGS_MAXIMUM)
	      {
		yycount = 1;
		yysize = yysize0;
		yyformat[sizeof yyunexpected - 1] = '\0';
		break;
	      }
	    yyarg[yycount++] = yytname[yyx];
	    yysize1 = yysize + yytnamerr (0, yytname[yyx]);
	    yysize_overflow |= (yysize1 < yysize);
	    yysize = yysize1;
	    yyfmt = yystpcpy (yyfmt, yyprefix);
	    yyprefix = yyor;
	  }

      yyf = YY_(yyformat);
      yysize1 = yysize + yystrlen (yyf);
      yysize_overflow |= (yysize1 < yysize);
      yysize = yysize1;

      if (yysize_overflow)
	return YYSIZE_MAXIMUM;

      if (yyresult)
	{
	  /* Avoid sprintf, as that infringes on the user's name space.
	     Don't have undefined behavior even if the translation
	     produced a string with the wrong number of "%s"s.  */
	  char *yyp = yyresult;
	  int yyi = 0;
	  while ((*yyp = *yyf) != '\0')
	    {
	      if (*yyp == '%' && yyf[1] == 's' && yyi < yycount)
		{
		  yyp += yytnamerr (yyp, yyarg[yyi++]);
		  yyf += 2;
		}
	      else
		{
		  yyp++;
		  yyf++;
		}
	    }
	}
      return yysize;
    }
}
#endif /* YYERROR_VERBOSE */


/*-----------------------------------------------.
| Release the memory associated to this symbol.  |
`-----------------------------------------------*/

/*ARGSUSED*/
#if (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
static void
yydestruct (const char *yymsg, int yytype, YYSTYPE *yyvaluep)
#else
static void
yydestruct (yymsg, yytype, yyvaluep)
    const char *yymsg;
    int yytype;
    YYSTYPE *yyvaluep;
#endif
{
  YYUSE (yyvaluep);

  if (!yymsg)
    yymsg = "Deleting";
  YY_SYMBOL_PRINT (yymsg, yytype, yyvaluep, yylocationp);

  switch (yytype)
    {

      default:
	break;
    }
}

/* Prevent warnings from -Wmissing-prototypes.  */
#ifdef YYPARSE_PARAM
#if defined __STDC__ || defined __cplusplus
int yyparse (void *YYPARSE_PARAM);
#else
int yyparse ();
#endif
#else /* ! YYPARSE_PARAM */
#if defined __STDC__ || defined __cplusplus
int yyparse (void);
#else
int yyparse ();
#endif
#endif /* ! YYPARSE_PARAM */


/* The lookahead symbol.  */
int yychar;

/* The semantic value of the lookahead symbol.  */
YYSTYPE yylval;

/* Number of syntax errors so far.  */
int yynerrs;



/*-------------------------.
| yyparse or yypush_parse.  |
`-------------------------*/

#ifdef YYPARSE_PARAM
#if (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
int
yyparse (void *YYPARSE_PARAM)
#else
int
yyparse (YYPARSE_PARAM)
    void *YYPARSE_PARAM;
#endif
#else /* ! YYPARSE_PARAM */
#if (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
int
yyparse (void)
#else
int
yyparse ()

#endif
#endif
{


    int yystate;
    /* Number of tokens to shift before error messages enabled.  */
    int yyerrstatus;

    /* The stacks and their tools:
       `yyss': related to states.
       `yyvs': related to semantic values.

       Refer to the stacks thru separate pointers, to allow yyoverflow
       to reallocate them elsewhere.  */

    /* The state stack.  */
    yytype_int16 yyssa[YYINITDEPTH];
    yytype_int16 *yyss;
    yytype_int16 *yyssp;

    /* The semantic value stack.  */
    YYSTYPE yyvsa[YYINITDEPTH];
    YYSTYPE *yyvs;
    YYSTYPE *yyvsp;

    YYSIZE_T yystacksize;

  int yyn;
  int yyresult;
  /* Lookahead token as an internal (translated) token number.  */
  int yytoken;
  /* The variables used to return semantic value and location from the
     action routines.  */
  YYSTYPE yyval;

#if YYERROR_VERBOSE
  /* Buffer for error messages, and its allocated size.  */
  char yymsgbuf[128];
  char *yymsg = yymsgbuf;
  YYSIZE_T yymsg_alloc = sizeof yymsgbuf;
#endif

#define YYPOPSTACK(N)   (yyvsp -= (N), yyssp -= (N))

  /* The number of symbols on the RHS of the reduced rule.
     Keep to zero when no symbol should be popped.  */
  int yylen = 0;

  yytoken = 0;
  yyss = yyssa;
  yyvs = yyvsa;
  yystacksize = YYINITDEPTH;

  YYDPRINTF ((stderr, "Starting parse\n"));

  yystate = 0;
  yyerrstatus = 0;
  yynerrs = 0;
  yychar = YYEMPTY; /* Cause a token to be read.  */

  /* Initialize stack pointers.
     Waste one element of value and location stack
     so that they stay on the same level as the state stack.
     The wasted elements are never initialized.  */
  yyssp = yyss;
  yyvsp = yyvs;

  goto yysetstate;

/*------------------------------------------------------------.
| yynewstate -- Push a new state, which is found in yystate.  |
`------------------------------------------------------------*/
 yynewstate:
  /* In all cases, when you get here, the value and location stacks
     have just been pushed.  So pushing a state here evens the stacks.  */
  yyssp++;

 yysetstate:
  *yyssp = yystate;

  if (yyss + yystacksize - 1 <= yyssp)
    {
      /* Get the current used size of the three stacks, in elements.  */
      YYSIZE_T yysize = yyssp - yyss + 1;

#ifdef yyoverflow
      {
	/* Give user a chance to reallocate the stack.  Use copies of
	   these so that the &'s don't force the real ones into
	   memory.  */
	YYSTYPE *yyvs1 = yyvs;
	yytype_int16 *yyss1 = yyss;

	/* Each stack pointer address is followed by the size of the
	   data in use in that stack, in bytes.  This used to be a
	   conditional around just the two extra args, but that might
	   be undefined if yyoverflow is a macro.  */
	yyoverflow (YY_("memory exhausted"),
		    &yyss1, yysize * sizeof (*yyssp),
		    &yyvs1, yysize * sizeof (*yyvsp),
		    &yystacksize);

	yyss = yyss1;
	yyvs = yyvs1;
      }
#else /* no yyoverflow */
# ifndef YYSTACK_RELOCATE
      goto yyexhaustedlab;
# else
      /* Extend the stack our own way.  */
      if (YYMAXDEPTH <= yystacksize)
	goto yyexhaustedlab;
      yystacksize *= 2;
      if (YYMAXDEPTH < yystacksize)
	yystacksize = YYMAXDEPTH;

      {
	yytype_int16 *yyss1 = yyss;
	union yyalloc *yyptr =
	  (union yyalloc *) YYSTACK_ALLOC (YYSTACK_BYTES (yystacksize));
	if (! yyptr)
	  goto yyexhaustedlab;
	YYSTACK_RELOCATE (yyss_alloc, yyss);
	YYSTACK_RELOCATE (yyvs_alloc, yyvs);
#  undef YYSTACK_RELOCATE
	if (yyss1 != yyssa)
	  YYSTACK_FREE (yyss1);
      }
# endif
#endif /* no yyoverflow */

      yyssp = yyss + yysize - 1;
      yyvsp = yyvs + yysize - 1;

      YYDPRINTF ((stderr, "Stack size increased to %lu\n",
		  (unsigned long int) yystacksize));

      if (yyss + yystacksize - 1 <= yyssp)
	YYABORT;
    }

  YYDPRINTF ((stderr, "Entering state %d\n", yystate));

  if (yystate == YYFINAL)
    YYACCEPT;

  goto yybackup;

/*-----------.
| yybackup.  |
`-----------*/
yybackup:

  /* Do appropriate processing given the current state.  Read a
     lookahead token if we need one and don't already have one.  */

  /* First try to decide what to do without reference to lookahead token.  */
  yyn = yypact[yystate];
  if (yyn == YYPACT_NINF)
    goto yydefault;

  /* Not known => get a lookahead token if don't already have one.  */

  /* YYCHAR is either YYEMPTY or YYEOF or a valid lookahead symbol.  */
  if (yychar == YYEMPTY)
    {
      YYDPRINTF ((stderr, "Reading a token: "));
      yychar = YYLEX;
    }

  if (yychar <= YYEOF)
    {
      yychar = yytoken = YYEOF;
      YYDPRINTF ((stderr, "Now at end of input.\n"));
    }
  else
    {
      yytoken = YYTRANSLATE (yychar);
      YY_SYMBOL_PRINT ("Next token is", yytoken, &yylval, &yylloc);
    }

  /* If the proper action on seeing token YYTOKEN is to reduce or to
     detect an error, take that action.  */
  yyn += yytoken;
  if (yyn < 0 || YYLAST < yyn || yycheck[yyn] != yytoken)
    goto yydefault;
  yyn = yytable[yyn];
  if (yyn <= 0)
    {
      if (yyn == 0 || yyn == YYTABLE_NINF)
	goto yyerrlab;
      yyn = -yyn;
      goto yyreduce;
    }

  /* Count tokens shifted since error; after three, turn off error
     status.  */
  if (yyerrstatus)
    yyerrstatus--;

  /* Shift the lookahead token.  */
  YY_SYMBOL_PRINT ("Shifting", yytoken, &yylval, &yylloc);

  /* Discard the shifted token.  */
  yychar = YYEMPTY;

  yystate = yyn;
  *++yyvsp = yylval;

  goto yynewstate;


/*-----------------------------------------------------------.
| yydefault -- do the default action for the current state.  |
`-----------------------------------------------------------*/
yydefault:
  yyn = yydefact[yystate];
  if (yyn == 0)
    goto yyerrlab;
  goto yyreduce;


/*-----------------------------.
| yyreduce -- Do a reduction.  |
`-----------------------------*/
yyreduce:
  /* yyn is the number of a rule to reduce with.  */
  yylen = yyr2[yyn];

  /* If YYLEN is nonzero, implement the default value of the action:
     `$$ = $1'.

     Otherwise, the following line sets YYVAL to garbage.
     This behavior is undocumented and Bison
     users should not rely upon it.  Assigning to YYVAL
     unconditionally makes the parser a bit smaller, and it avoids a
     GCC warning that YYVAL may be used uninitialized.  */
  yyval = yyvsp[1-yylen];


  YY_REDUCE_PRINT (yyn);
  switch (yyn)
    {
        case 73:

/* Line 1455 of yacc.c  */
#line 355 "src/cfgparser/oparse.y"
    {
  struct olsr_if *in = olsr_cnf->interfaces;
  printf("\nInterface Defaults");
  /*remove Interface Defaults from Interface list as they are no interface!*/
  olsr_cnf->interfaces = in->next;
  ifs_in_curr_cfg=0;
  /*free interface but keep its config intact?*/
  free(in->cnfi);
  free(in);

;}
    break;

  case 99:

/* Line 1455 of yacc.c  */
#line 402 "src/cfgparser/oparse.y"
    {
  struct olsr_if *in = malloc(sizeof(*in));

  if (in == NULL) {
    fprintf(stderr, "Out of memory(ADD IF)\n");
    YYABORT;
  }

  in->cnf = get_default_if_config();
  in->cnfi = get_default_if_config();

  if (in->cnf == NULL || in->cnfi == NULL) {
    fprintf(stderr, "Out of memory(ADD DEFIFRULE)\n");
    YYABORT;
  }

  //should not need a name any more, as we free it on "}" again
  //in->name = strdup(interface_defaults_name);

  olsr_cnf->interface_defaults = in->cnf;

  /* Queue */
  in->next = olsr_cnf->interfaces;
  olsr_cnf->interfaces = in;
  ifs_in_curr_cfg=1;
  
  fflush(stdout);
;}
    break;

  case 100:

/* Line 1455 of yacc.c  */
#line 433 "src/cfgparser/oparse.y"
    {
  olsr_cnf->ipc_connections = (yyvsp[(2) - (2)])->integer;
  free((yyvsp[(2) - (2)]));
;}
    break;

  case 101:

/* Line 1455 of yacc.c  */
#line 440 "src/cfgparser/oparse.y"
    {
  union olsr_ip_addr ipaddr;
  PARSER_DEBUG_PRINTF("\tIPC host: %s\n", (yyvsp[(2) - (2)])->string);
  
  if (inet_aton((yyvsp[(2) - (2)])->string, &ipaddr.v4) == 0) {
    fprintf(stderr, "Failed converting IP address IPC %s\n", (yyvsp[(2) - (2)])->string);
    YYABORT;
  }

  ip_prefix_list_add(&olsr_cnf->ipc_nets, &ipaddr, olsr_cnf->maxplen);

  free((yyvsp[(2) - (2)])->string);
  free((yyvsp[(2) - (2)]));
;}
    break;

  case 102:

/* Line 1455 of yacc.c  */
#line 457 "src/cfgparser/oparse.y"
    {
  union olsr_ip_addr ipaddr, netmask;

  PARSER_DEBUG_PRINTF("\tIPC net: %s/%s\n", (yyvsp[(2) - (3)])->string, (yyvsp[(3) - (3)])->string);
  
  if (inet_pton(AF_INET, (yyvsp[(2) - (3)])->string, &ipaddr.v4) == 0) {
    fprintf(stderr, "Failed converting IP net IPC %s\n", (yyvsp[(2) - (3)])->string);
    YYABORT;
  }

  if (inet_pton(AF_INET, (yyvsp[(3) - (3)])->string, &netmask.v4) == 0) {
    fprintf(stderr, "Failed converting IP mask IPC %s\n", (yyvsp[(3) - (3)])->string);
    YYABORT;
  }

  ip_prefix_list_add(&olsr_cnf->ipc_nets, &ipaddr, olsr_netmask_to_prefix(&netmask));

  free((yyvsp[(2) - (3)])->string);
  free((yyvsp[(2) - (3)]));
  free((yyvsp[(3) - (3)])->string);
  free((yyvsp[(3) - (3)]));
;}
    break;

  case 103:

/* Line 1455 of yacc.c  */
#line 480 "src/cfgparser/oparse.y"
    {
  union olsr_ip_addr ipaddr;

  PARSER_DEBUG_PRINTF("\tIPC net: %s/%s\n", (yyvsp[(2) - (4)])->string, (yyvsp[(3) - (4)])->string);
  
  if (inet_pton(AF_INET, (yyvsp[(2) - (4)])->string, &ipaddr.v4) == 0) {
    fprintf(stderr, "Failed converting IP net IPC %s\n", (yyvsp[(2) - (4)])->string);
    YYABORT;
  }

  if ((yyvsp[(4) - (4)])->integer > olsr_cnf->maxplen) {
    fprintf(stderr, "ipcnet: Prefix len %u > %d is not allowed!\n", (yyvsp[(4) - (4)])->integer, olsr_cnf->maxplen);
    YYABORT;
  }

  ip_prefix_list_add(&olsr_cnf->ipc_nets, &ipaddr, (yyvsp[(4) - (4)])->integer);

  free((yyvsp[(2) - (4)])->string);
  free((yyvsp[(2) - (4)]));
  free((yyvsp[(4) - (4)]));
;}
    break;

  case 104:

/* Line 1455 of yacc.c  */
#line 504 "src/cfgparser/oparse.y"
    {
  int ifcnt = ifs_in_curr_cfg;
  struct olsr_if *ifs = olsr_cnf->interfaces;

  PARSER_DEBUG_PRINTF("Fixed willingness: %d\n", (yyvsp[(2) - (2)])->integer);

  while (ifcnt) {
    ifs->cnf->weight.value = (yyvsp[(2) - (2)])->integer;
    ifs->cnf->weight.fixed = true;
    ifs->cnfi->weight.value = (yyvsp[(2) - (2)])->integer;
    ifs->cnfi->weight.fixed = true;

    ifs = ifs->next;
    ifcnt--;
  }

  free((yyvsp[(2) - (2)]));
;}
    break;

  case 105:

/* Line 1455 of yacc.c  */
#line 525 "src/cfgparser/oparse.y"
    {
  int ifcnt = ifs_in_curr_cfg;
  struct olsr_if *ifs = olsr_cnf->interfaces;
	int mode = (strcmp((yyvsp[(2) - (2)])->string, "ether") == 0)?IF_MODE_ETHER:IF_MODE_MESH;

  PARSER_DEBUG_PRINTF("\tMode: %s\n", (yyvsp[(2) - (2)])->string);

	SET_IFS_CONF(ifs, ifcnt, mode, mode);
	
  free((yyvsp[(2) - (2)])->string);
  free((yyvsp[(2) - (2)]));
;}
    break;

  case 106:

/* Line 1455 of yacc.c  */
#line 540 "src/cfgparser/oparse.y"
    {
  struct in_addr in;
  int ifcnt = ifs_in_curr_cfg;
  struct olsr_if *ifs = olsr_cnf->interfaces;

  PARSER_DEBUG_PRINTF("\tIPv4 broadcast: %s\n", (yyvsp[(2) - (2)])->string);

  if (inet_aton((yyvsp[(2) - (2)])->string, &in) == 0) {
    fprintf(stderr, "isetipv4br: Failed converting IP address %s\n", (yyvsp[(2) - (2)])->string);
    YYABORT;
  }

	SET_IFS_CONF(ifs, ifcnt, ipv4_multicast.v4, in);

  free((yyvsp[(2) - (2)])->string);
  free((yyvsp[(2) - (2)]));
;}
    break;

  case 107:

/* Line 1455 of yacc.c  */
#line 560 "src/cfgparser/oparse.y"
    {
  struct in_addr in;
  int ifcnt = ifs_in_curr_cfg;
  struct olsr_if *ifs = olsr_cnf->interfaces;

  PARSER_DEBUG_PRINTF("\tIPv4 broadcast: %s\n", (yyvsp[(2) - (2)])->string);

  if (inet_aton((yyvsp[(2) - (2)])->string, &in) == 0) {
    fprintf(stderr, "isetipv4br: Failed converting IP address %s\n", (yyvsp[(2) - (2)])->string);
    YYABORT;
  }

	SET_IFS_CONF(ifs, ifcnt, ipv4_multicast.v4, in);

  free((yyvsp[(2) - (2)])->string);
  free((yyvsp[(2) - (2)]));
;}
    break;

  case 108:

/* Line 1455 of yacc.c  */
#line 580 "src/cfgparser/oparse.y"
    {
  struct in6_addr in6;
  int ifcnt = ifs_in_curr_cfg;
  struct olsr_if *ifs = olsr_cnf->interfaces;

  PARSER_DEBUG_PRINTF("\tIPv6 multicast: %s\n", (yyvsp[(2) - (2)])->string);

  if (inet_pton(AF_INET6, (yyvsp[(2) - (2)])->string, &in6) <= 0) {
    fprintf(stderr, "isetipv6mc: Failed converting IP address %s\n", (yyvsp[(2) - (2)])->string);
    YYABORT;
  }

	SET_IFS_CONF(ifs, ifcnt, ipv6_multicast.v6, in6);

  free((yyvsp[(2) - (2)])->string);
  free((yyvsp[(2) - (2)]));
;}
    break;

  case 109:

/* Line 1455 of yacc.c  */
#line 600 "src/cfgparser/oparse.y"
    {
  struct in_addr in;
  int ifcnt = ifs_in_curr_cfg;
  struct olsr_if *ifs = olsr_cnf->interfaces;

  PARSER_DEBUG_PRINTF("\tIPv4 src: %s\n", (yyvsp[(2) - (2)])->string);

  if (inet_aton((yyvsp[(2) - (2)])->string, &in) == 0) {
    fprintf(stderr, "isetipv4src: Failed converting IP address %s\n", (yyvsp[(2) - (2)])->string);
    YYABORT;
  }

	SET_IFS_CONF(ifs, ifcnt, ipv4_src.v4, in);

  free((yyvsp[(2) - (2)])->string);
  free((yyvsp[(2) - (2)]));
;}
    break;

  case 110:

/* Line 1455 of yacc.c  */
#line 620 "src/cfgparser/oparse.y"
    {
  struct olsr_ip_prefix pr6;
  int ifcnt = ifs_in_curr_cfg;
  struct olsr_if *ifs = olsr_cnf->interfaces;

  PARSER_DEBUG_PRINTF("\tIPv6 src prefix: %s\n", (yyvsp[(2) - (2)])->string);

  if (olsr_string_to_prefix(AF_INET6, &pr6, (yyvsp[(2) - (2)])->string) <= 0) {
    fprintf(stderr, "isetipv6src: Failed converting IP prefix %s\n", (yyvsp[(2) - (2)])->string);
    YYABORT;
  }

	SET_IFS_CONF(ifs, ifcnt, ipv6_src, pr6);

  free((yyvsp[(2) - (2)])->string);
  free((yyvsp[(2) - (2)]));
;}
    break;

  case 111:

/* Line 1455 of yacc.c  */
#line 640 "src/cfgparser/oparse.y"
    {
  int ifcnt = ifs_in_curr_cfg;
  struct olsr_if *ifs = olsr_cnf->interfaces;

  PARSER_DEBUG_PRINTF("\tHELLO interval: %0.2f\n", (yyvsp[(2) - (2)])->floating);

	SET_IFS_CONF(ifs, ifcnt, hello_params.emission_interval, (yyvsp[(2) - (2)])->floating);

  free((yyvsp[(2) - (2)]));
;}
    break;

  case 112:

/* Line 1455 of yacc.c  */
#line 652 "src/cfgparser/oparse.y"
    {
  int ifcnt = ifs_in_curr_cfg;
  struct olsr_if *ifs = olsr_cnf->interfaces;

  PARSER_DEBUG_PRINTF("\tHELLO validity: %0.2f\n", (yyvsp[(2) - (2)])->floating);

	SET_IFS_CONF(ifs, ifcnt, hello_params.validity_time, (yyvsp[(2) - (2)])->floating);

  free((yyvsp[(2) - (2)]));
;}
    break;

  case 113:

/* Line 1455 of yacc.c  */
#line 664 "src/cfgparser/oparse.y"
    {
  int ifcnt = ifs_in_curr_cfg;
  struct olsr_if *ifs = olsr_cnf->interfaces;

  PARSER_DEBUG_PRINTF("\tTC interval: %0.2f\n", (yyvsp[(2) - (2)])->floating);

	SET_IFS_CONF(ifs, ifcnt, tc_params.emission_interval, (yyvsp[(2) - (2)])->floating);

  free((yyvsp[(2) - (2)]));
;}
    break;

  case 114:

/* Line 1455 of yacc.c  */
#line 676 "src/cfgparser/oparse.y"
    {
  int ifcnt = ifs_in_curr_cfg;
  struct olsr_if *ifs = olsr_cnf->interfaces;
  
  PARSER_DEBUG_PRINTF("\tTC validity: %0.2f\n", (yyvsp[(2) - (2)])->floating);
  
 SET_IFS_CONF(ifs, ifcnt, tc_params.validity_time, (yyvsp[(2) - (2)])->floating);

  free((yyvsp[(2) - (2)]));
;}
    break;

  case 115:

/* Line 1455 of yacc.c  */
#line 688 "src/cfgparser/oparse.y"
    {
  int ifcnt = ifs_in_curr_cfg;
  struct olsr_if *ifs = olsr_cnf->interfaces;


  PARSER_DEBUG_PRINTF("\tMID interval: %0.2f\n", (yyvsp[(2) - (2)])->floating);
  
  SET_IFS_CONF(ifs, ifcnt, mid_params.emission_interval, (yyvsp[(2) - (2)])->floating);

  free((yyvsp[(2) - (2)]));
;}
    break;

  case 116:

/* Line 1455 of yacc.c  */
#line 701 "src/cfgparser/oparse.y"
    {
  int ifcnt = ifs_in_curr_cfg;
  struct olsr_if *ifs = olsr_cnf->interfaces;

  PARSER_DEBUG_PRINTF("\tMID validity: %0.2f\n", (yyvsp[(2) - (2)])->floating);
  
  SET_IFS_CONF(ifs, ifcnt, mid_params.validity_time, (yyvsp[(2) - (2)])->floating);

  free((yyvsp[(2) - (2)]));
;}
    break;

  case 117:

/* Line 1455 of yacc.c  */
#line 713 "src/cfgparser/oparse.y"
    {
  int ifcnt = ifs_in_curr_cfg;
  struct olsr_if *ifs = olsr_cnf->interfaces;
  
  PARSER_DEBUG_PRINTF("\tHNA interval: %0.2f\n", (yyvsp[(2) - (2)])->floating);

  SET_IFS_CONF(ifs, ifcnt, hna_params.emission_interval, (yyvsp[(2) - (2)])->floating);

  free((yyvsp[(2) - (2)]));
;}
    break;

  case 118:

/* Line 1455 of yacc.c  */
#line 725 "src/cfgparser/oparse.y"
    {
  int ifcnt = ifs_in_curr_cfg;
  struct olsr_if *ifs = olsr_cnf->interfaces;

  PARSER_DEBUG_PRINTF("\tHNA validity: %0.2f\n", (yyvsp[(2) - (2)])->floating);

  SET_IFS_CONF(ifs, ifcnt, hna_params.validity_time, (yyvsp[(2) - (2)])->floating);

  free((yyvsp[(2) - (2)]));
;}
    break;

  case 119:

/* Line 1455 of yacc.c  */
#line 737 "src/cfgparser/oparse.y"
    {
  int ifcnt = ifs_in_curr_cfg;
  struct olsr_if *ifs = olsr_cnf->interfaces;

  PARSER_DEBUG_PRINTF("\tAutodetect changes: %s\n", (yyvsp[(2) - (2)])->boolean ? "YES" : "NO");

  SET_IFS_CONF(ifs, ifcnt, autodetect_chg, (yyvsp[(2) - (2)])->boolean);

  free((yyvsp[(2) - (2)]));
;}
    break;

  case 120:

/* Line 1455 of yacc.c  */
#line 750 "src/cfgparser/oparse.y"
    {
  if (lq_mult_helper((yyvsp[(2) - (3)]), (yyvsp[(3) - (3)])) < 0) {
    YYABORT;
  }
;}
    break;

  case 121:

/* Line 1455 of yacc.c  */
#line 757 "src/cfgparser/oparse.y"
    {
  if (lq_mult_helper((yyvsp[(2) - (3)]), (yyvsp[(3) - (3)])) < 0) {
    YYABORT;
  }
;}
    break;

  case 122:

/* Line 1455 of yacc.c  */
#line 764 "src/cfgparser/oparse.y"
    {
  if (lq_mult_helper((yyvsp[(2) - (3)]), (yyvsp[(3) - (3)])) < 0) {
    YYABORT;
  }
;}
    break;

  case 123:

/* Line 1455 of yacc.c  */
#line 772 "src/cfgparser/oparse.y"
    {
  olsr_cnf->debug_level = (yyvsp[(2) - (2)])->integer;
  PARSER_DEBUG_PRINTF("Debug level: %d\n", olsr_cnf->debug_level);
  free((yyvsp[(2) - (2)]));
;}
    break;

  case 124:

/* Line 1455 of yacc.c  */
#line 781 "src/cfgparser/oparse.y"
    {
  if ((yyvsp[(2) - (2)])->integer == 4) {
    olsr_cnf->ip_version = AF_INET;
    olsr_cnf->ipsize = sizeof(struct in_addr);
    olsr_cnf->maxplen = 32;
  } else if ((yyvsp[(2) - (2)])->integer == 6) {
    olsr_cnf->ip_version = AF_INET6;
    olsr_cnf->ipsize = sizeof(struct in6_addr);
    olsr_cnf->maxplen = 128;
  } else {
    fprintf(stderr, "IPversion must be 4 or 6!\n");
    YYABORT;
  }

  PARSER_DEBUG_PRINTF("IpVersion: %d\n", (yyvsp[(2) - (2)])->integer);
  free((yyvsp[(2) - (2)]));
;}
    break;

  case 125:

/* Line 1455 of yacc.c  */
#line 801 "src/cfgparser/oparse.y"
    {
  int i;
  PARSER_DEBUG_PRINTF("FIBMetric: %s\n", (yyvsp[(2) - (2)])->string);
  for (i=0; i<FIBM_CNT; i++) {
    if (strcmp((yyvsp[(2) - (2)])->string, FIB_METRIC_TXT[i]) == 0) {
      olsr_cnf->fib_metric = i;
      break;
    }
  }
  if (i == FIBM_CNT) {
    fprintf(stderr, "Bad FIBMetric value: %s\n", (yyvsp[(2) - (2)])->string);
    YYABORT;
  }
  free((yyvsp[(1) - (2)]));
  free((yyvsp[(2) - (2)])->string);
  free((yyvsp[(2) - (2)]));
;}
    break;

  case 126:

/* Line 1455 of yacc.c  */
#line 821 "src/cfgparser/oparse.y"
    {
  union olsr_ip_addr ipaddr, netmask;

  if (olsr_cnf->ip_version == AF_INET6) {
    fprintf(stderr, "IPv4 addresses can only be used if \"IpVersion\" == 4, skipping HNA.\n");
    olsr_startup_sleep(3);
  }
  else {
    PARSER_DEBUG_PRINTF("HNA IPv4 entry: %s/%s\n", (yyvsp[(1) - (2)])->string, (yyvsp[(2) - (2)])->string);

    if (inet_pton(AF_INET, (yyvsp[(1) - (2)])->string, &ipaddr.v4) <= 0) {
      fprintf(stderr, "ihna4entry: Failed converting IP address %s\n", (yyvsp[(1) - (2)])->string);
      YYABORT;
    }
    if (inet_pton(AF_INET, (yyvsp[(2) - (2)])->string, &netmask.v4) <= 0) {
      fprintf(stderr, "ihna4entry: Failed converting IP address %s\n", (yyvsp[(1) - (2)])->string);
      YYABORT;
    }

    /* check that the given IP address is actually a network address */
    if ((ipaddr.v4.s_addr & ~netmask.v4.s_addr) != 0) {
      fprintf(stderr, "ihna4entry: The ipaddress \"%s\" is not a network address!\n", (yyvsp[(1) - (2)])->string);
      YYABORT;
    }

    /* Queue */
    ip_prefix_list_add(&olsr_cnf->hna_entries, &ipaddr, olsr_netmask_to_prefix(&netmask));
  }
  free((yyvsp[(1) - (2)])->string);
  free((yyvsp[(1) - (2)]));
  free((yyvsp[(2) - (2)])->string);
  free((yyvsp[(2) - (2)]));
;}
    break;

  case 127:

/* Line 1455 of yacc.c  */
#line 855 "src/cfgparser/oparse.y"
    {
  union olsr_ip_addr ipaddr, netmask;

  if (olsr_cnf->ip_version == AF_INET6) {
    fprintf(stderr, "IPv4 addresses can only be used if \"IpVersion\" == 4, skipping HNA.\n");
    olsr_startup_sleep(3);
  }
  else {
    PARSER_DEBUG_PRINTF("HNA IPv4 entry: %s/%d\n", (yyvsp[(1) - (3)])->string, (yyvsp[(3) - (3)])->integer);

    if (inet_pton(AF_INET, (yyvsp[(1) - (3)])->string, &ipaddr.v4) <= 0) {
      fprintf(stderr, "ihna4entry: Failed converting IP address %s\n", (yyvsp[(1) - (3)])->string);
      YYABORT;
    }
    if ((yyvsp[(3) - (3)])->integer > olsr_cnf->maxplen) {
      fprintf(stderr, "ihna4entry: Prefix len %u > %d is not allowed!\n", (yyvsp[(3) - (3)])->integer, olsr_cnf->maxplen);
      YYABORT;
    }

    /* check that the given IP address is actually a network address */
    olsr_prefix_to_netmask(&netmask, (yyvsp[(3) - (3)])->integer);
    if ((ipaddr.v4.s_addr & ~netmask.v4.s_addr) != 0) {
      fprintf(stderr, "ihna4entry: The ipaddress \"%s\" is not a network address!\n", (yyvsp[(1) - (3)])->string);
      YYABORT;
    }

    /* Queue */
    ip_prefix_list_add(&olsr_cnf->hna_entries, &ipaddr, (yyvsp[(3) - (3)])->integer);
  }
  free((yyvsp[(1) - (3)])->string);
  free((yyvsp[(1) - (3)]));
  free((yyvsp[(3) - (3)]));
;}
    break;

  case 128:

/* Line 1455 of yacc.c  */
#line 891 "src/cfgparser/oparse.y"
    {
  if (add_ipv6_addr((yyvsp[(1) - (2)]), (yyvsp[(2) - (2)]))) {
    YYABORT;
  }
;}
    break;

  case 129:

/* Line 1455 of yacc.c  */
#line 897 "src/cfgparser/oparse.y"
    {
  if (add_ipv6_addr((yyvsp[(1) - (3)]), (yyvsp[(3) - (3)]))) {
    YYABORT;
  }
;}
    break;

  case 130:

/* Line 1455 of yacc.c  */
#line 905 "src/cfgparser/oparse.y"
    {
  PARSER_DEBUG_PRINTF("setting ifs_in_curr_cfg = 0\n");
  ifs_in_curr_cfg = 0;
;}
    break;

  case 131:

/* Line 1455 of yacc.c  */
#line 912 "src/cfgparser/oparse.y"
    {
  struct olsr_if *in, *last;
  in = olsr_cnf->interfaces;
  last = NULL;
  while (in != NULL) {
    if (strcmp(in->name, (yyvsp[(1) - (1)])->string) == 0) {
      free ((yyvsp[(1) - (1)])->string);
      break;
    }
    last = in;
    in = in->next;
  }

  if (in != NULL) {
    /* remove old interface from list to add it later at the beginning */
    if (last) {
      last->next = in->next;
    }
    else {
      olsr_cnf->interfaces = in->next;
    }
  }
  else {
    in = malloc(sizeof(*in));
    if (in == NULL) {
      fprintf(stderr, "Out of memory(ADD IF)\n");
      YYABORT;
    }
    memset(in, 0, sizeof(*in));

    in->cnf = malloc(sizeof(*in->cnf));
    if (in->cnf == NULL) {
      fprintf(stderr, "Out of memory(ADD IFRULE)\n");
      YYABORT;
    }
    memset(in->cnf, 0x00, sizeof(*in->cnf));

    in->cnfi = malloc(sizeof(*in->cnfi));
    if (in->cnf == NULL) {
      fprintf(stderr, "Out of memory(ADD IFRULE)\n");
      YYABORT;
    }
    memset(in->cnfi, 0xFF, sizeof(*in->cnfi));
    in->cnfi->orig_lq_mult_cnt=0;

    in->name = (yyvsp[(1) - (1)])->string;
  }
  /* Queue */
  in->next = olsr_cnf->interfaces;
  olsr_cnf->interfaces = in;
  ifs_in_curr_cfg++;
  free((yyvsp[(1) - (1)]));
;}
    break;

  case 132:

/* Line 1455 of yacc.c  */
#line 968 "src/cfgparser/oparse.y"
    {
  PARSER_DEBUG_PRINTF("Noint set to %d\n", (yyvsp[(2) - (2)])->boolean);
  olsr_cnf->allow_no_interfaces = (yyvsp[(2) - (2)])->boolean;
  free((yyvsp[(2) - (2)]));
;}
    break;

  case 133:

/* Line 1455 of yacc.c  */
#line 976 "src/cfgparser/oparse.y"
    {
  PARSER_DEBUG_PRINTF("TOS: %d\n", (yyvsp[(2) - (2)])->integer);
  olsr_cnf->tos = (yyvsp[(2) - (2)])->integer;
  free((yyvsp[(2) - (2)]));

;}
    break;

  case 134:

/* Line 1455 of yacc.c  */
#line 985 "src/cfgparser/oparse.y"
    {
  PARSER_DEBUG_PRINTF("OlsrPort: %d\n", (yyvsp[(2) - (2)])->integer);
  olsr_cnf->olsrport = (yyvsp[(2) - (2)])->integer;
  free((yyvsp[(2) - (2)]));
;}
    break;

  case 135:

/* Line 1455 of yacc.c  */
#line 993 "src/cfgparser/oparse.y"
    {
  PARSER_DEBUG_PRINTF("RtProto: %d\n", (yyvsp[(2) - (2)])->integer);
  olsr_cnf->rt_proto = (yyvsp[(2) - (2)])->integer;
  free((yyvsp[(2) - (2)]));
;}
    break;

  case 136:

/* Line 1455 of yacc.c  */
#line 1001 "src/cfgparser/oparse.y"
    {
  PARSER_DEBUG_PRINTF("RtTable: %d\n", (yyvsp[(2) - (2)])->integer);
  olsr_cnf->rt_table = (yyvsp[(2) - (2)])->integer;
  free((yyvsp[(2) - (2)]));
;}
    break;

  case 137:

/* Line 1455 of yacc.c  */
#line 1007 "src/cfgparser/oparse.y"
    {
  PARSER_DEBUG_PRINTF("RtTable: auto\n");
  olsr_cnf->rt_table = DEF_RT_AUTO;
  free((yyvsp[(2) - (2)]));
;}
    break;

  case 138:

/* Line 1455 of yacc.c  */
#line 1015 "src/cfgparser/oparse.y"
    {
  PARSER_DEBUG_PRINTF("RtTableDefault: %d\n", (yyvsp[(2) - (2)])->integer);
  olsr_cnf->rt_table_default = (yyvsp[(2) - (2)])->integer;
  free((yyvsp[(2) - (2)]));
;}
    break;

  case 139:

/* Line 1455 of yacc.c  */
#line 1021 "src/cfgparser/oparse.y"
    {
  PARSER_DEBUG_PRINTF("RtTableDefault: auto\n");
  olsr_cnf->rt_table_default = DEF_RT_AUTO;
  free((yyvsp[(2) - (2)]));
;}
    break;

  case 140:

/* Line 1455 of yacc.c  */
#line 1029 "src/cfgparser/oparse.y"
    {
  PARSER_DEBUG_PRINTF("RtTableTunnel: %d\n", (yyvsp[(2) - (2)])->integer);
  olsr_cnf->rt_table_tunnel = (yyvsp[(2) - (2)])->integer;
  free((yyvsp[(2) - (2)]));
;}
    break;

  case 141:

/* Line 1455 of yacc.c  */
#line 1035 "src/cfgparser/oparse.y"
    {
  PARSER_DEBUG_PRINTF("RtTableTunnel: auto\n");
  olsr_cnf->rt_table_tunnel = DEF_RT_AUTO;
  free((yyvsp[(2) - (2)]));
;}
    break;

  case 142:

/* Line 1455 of yacc.c  */
#line 1043 "src/cfgparser/oparse.y"
    {
  PARSER_DEBUG_PRINTF("RtTablePriority: %d\n", (yyvsp[(2) - (2)])->integer);
  olsr_cnf->rt_table_pri = (yyvsp[(2) - (2)])->integer;
  free((yyvsp[(2) - (2)]));
;}
    break;

  case 143:

/* Line 1455 of yacc.c  */
#line 1049 "src/cfgparser/oparse.y"
    {
  PARSER_DEBUG_PRINTF("RtTablePriority: auto\n");
  olsr_cnf->rt_table_pri = DEF_RT_AUTO;
  free((yyvsp[(2) - (2)]));
;}
    break;

  case 144:

/* Line 1455 of yacc.c  */
#line 1055 "src/cfgparser/oparse.y"
    {
  PARSER_DEBUG_PRINTF("RtTablePriority: none\n");
  olsr_cnf->rt_table_pri = DEF_RT_NONE;
  free((yyvsp[(2) - (2)]));
;}
    break;

  case 145:

/* Line 1455 of yacc.c  */
#line 1063 "src/cfgparser/oparse.y"
    {
  PARSER_DEBUG_PRINTF("RtTableDefaultPriority: %d\n", (yyvsp[(2) - (2)])->integer);
  olsr_cnf->rt_table_default_pri = (yyvsp[(2) - (2)])->integer;
  free((yyvsp[(2) - (2)]));
;}
    break;

  case 146:

/* Line 1455 of yacc.c  */
#line 1069 "src/cfgparser/oparse.y"
    {
  PARSER_DEBUG_PRINTF("RtTableDefaultPriority: auto\n");
  olsr_cnf->rt_table_default_pri = DEF_RT_AUTO;
  free((yyvsp[(2) - (2)]));
;}
    break;

  case 147:

/* Line 1455 of yacc.c  */
#line 1075 "src/cfgparser/oparse.y"
    {
  PARSER_DEBUG_PRINTF("RtTableDefaultPriority: none\n");
  olsr_cnf->rt_table_default_pri = DEF_RT_NONE;
  free((yyvsp[(2) - (2)]));
;}
    break;

  case 148:

/* Line 1455 of yacc.c  */
#line 1083 "src/cfgparser/oparse.y"
    {
  PARSER_DEBUG_PRINTF("RtTableTunnelPriority: %d\n", (yyvsp[(2) - (2)])->integer);
  olsr_cnf->rt_table_tunnel_pri = (yyvsp[(2) - (2)])->integer;
  free((yyvsp[(2) - (2)]));
;}
    break;

  case 149:

/* Line 1455 of yacc.c  */
#line 1089 "src/cfgparser/oparse.y"
    {
  PARSER_DEBUG_PRINTF("RtTableTunnelPriority: auto\n");
  olsr_cnf->rt_table_tunnel_pri = DEF_RT_AUTO;
  free((yyvsp[(2) - (2)]));
;}
    break;

  case 150:

/* Line 1455 of yacc.c  */
#line 1095 "src/cfgparser/oparse.y"
    {
  PARSER_DEBUG_PRINTF("RtTableTunnelPriority: none\n");
  olsr_cnf->rt_table_tunnel_pri = DEF_RT_NONE;
  free((yyvsp[(2) - (2)]));
;}
    break;

  case 151:

/* Line 1455 of yacc.c  */
#line 1103 "src/cfgparser/oparse.y"
    {
  PARSER_DEBUG_PRINTF("RtTableDefaultOlsrPriority: %d\n", (yyvsp[(2) - (2)])->integer);
  olsr_cnf->rt_table_defaultolsr_pri = (yyvsp[(2) - (2)])->integer;
  free((yyvsp[(2) - (2)]));
;}
    break;

  case 152:

/* Line 1455 of yacc.c  */
#line 1109 "src/cfgparser/oparse.y"
    {
  PARSER_DEBUG_PRINTF("RtTableDefaultOlsrPriority: auto\n");
  olsr_cnf->rt_table_defaultolsr_pri = DEF_RT_AUTO;
  free((yyvsp[(2) - (2)]));
;}
    break;

  case 153:

/* Line 1455 of yacc.c  */
#line 1115 "src/cfgparser/oparse.y"
    {
  PARSER_DEBUG_PRINTF("RtTableDefaultOlsrPriority: none\n");
  olsr_cnf->rt_table_defaultolsr_pri = DEF_RT_NONE;
  free((yyvsp[(2) - (2)]));
;}
    break;

  case 154:

/* Line 1455 of yacc.c  */
#line 1123 "src/cfgparser/oparse.y"
    {
  PARSER_DEBUG_PRINTF("Willingness: %d\n", (yyvsp[(2) - (2)])->integer);
  olsr_cnf->willingness_auto = false;
  olsr_cnf->willingness = (yyvsp[(2) - (2)])->integer;
  free((yyvsp[(2) - (2)]));
;}
    break;

  case 155:

/* Line 1455 of yacc.c  */
#line 1132 "src/cfgparser/oparse.y"
    {
  olsr_cnf->use_hysteresis = (yyvsp[(2) - (2)])->boolean;
  PARSER_DEBUG_PRINTF("Hysteresis %s\n", olsr_cnf->use_hysteresis ? "enabled" : "disabled");
  free((yyvsp[(2) - (2)]));
;}
    break;

  case 156:

/* Line 1455 of yacc.c  */
#line 1140 "src/cfgparser/oparse.y"
    {
  olsr_cnf->hysteresis_param.scaling = (yyvsp[(2) - (2)])->floating;
  PARSER_DEBUG_PRINTF("Hysteresis Scaling: %0.2f\n", (yyvsp[(2) - (2)])->floating);
  free((yyvsp[(2) - (2)]));
;}
    break;

  case 157:

/* Line 1455 of yacc.c  */
#line 1148 "src/cfgparser/oparse.y"
    {
  olsr_cnf->hysteresis_param.thr_high = (yyvsp[(2) - (2)])->floating;
  PARSER_DEBUG_PRINTF("Hysteresis UpperThr: %0.2f\n", (yyvsp[(2) - (2)])->floating);
  free((yyvsp[(2) - (2)]));
;}
    break;

  case 158:

/* Line 1455 of yacc.c  */
#line 1156 "src/cfgparser/oparse.y"
    {
  olsr_cnf->hysteresis_param.thr_low = (yyvsp[(2) - (2)])->floating;
  PARSER_DEBUG_PRINTF("Hysteresis LowerThr: %0.2f\n", (yyvsp[(2) - (2)])->floating);
  free((yyvsp[(2) - (2)]));
;}
    break;

  case 159:

/* Line 1455 of yacc.c  */
#line 1164 "src/cfgparser/oparse.y"
    {
  PARSER_DEBUG_PRINTF("Pollrate %0.2f\n", (yyvsp[(2) - (2)])->floating);
  olsr_cnf->pollrate = (yyvsp[(2) - (2)])->floating;
  free((yyvsp[(2) - (2)]));
;}
    break;

  case 160:

/* Line 1455 of yacc.c  */
#line 1172 "src/cfgparser/oparse.y"
    {
  PARSER_DEBUG_PRINTF("NIC Changes Pollrate %0.2f\n", (yyvsp[(2) - (2)])->floating);
  olsr_cnf->nic_chgs_pollrate = (yyvsp[(2) - (2)])->floating;
  free((yyvsp[(2) - (2)]));
;}
    break;

  case 161:

/* Line 1455 of yacc.c  */
#line 1180 "src/cfgparser/oparse.y"
    {
  PARSER_DEBUG_PRINTF("TC redundancy %d\n", (yyvsp[(2) - (2)])->integer);
  olsr_cnf->tc_redundancy = (yyvsp[(2) - (2)])->integer;
  free((yyvsp[(2) - (2)]));
;}
    break;

  case 162:

/* Line 1455 of yacc.c  */
#line 1188 "src/cfgparser/oparse.y"
    {
  PARSER_DEBUG_PRINTF("MPR coverage %d\n", (yyvsp[(2) - (2)])->integer);
  olsr_cnf->mpr_coverage = (yyvsp[(2) - (2)])->integer;
  free((yyvsp[(2) - (2)]));
;}
    break;

  case 163:

/* Line 1455 of yacc.c  */
#line 1196 "src/cfgparser/oparse.y"
    {
  PARSER_DEBUG_PRINTF("Link quality level %d\n", (yyvsp[(2) - (2)])->integer);
  olsr_cnf->lq_level = (yyvsp[(2) - (2)])->integer;
  free((yyvsp[(2) - (2)]));
;}
    break;

  case 164:

/* Line 1455 of yacc.c  */
#line 1204 "src/cfgparser/oparse.y"
    {
  PARSER_DEBUG_PRINTF("Link quality fish eye %d\n", (yyvsp[(2) - (2)])->integer);
  olsr_cnf->lq_fish = (yyvsp[(2) - (2)])->integer;
  free((yyvsp[(2) - (2)]));
;}
    break;

  case 165:

/* Line 1455 of yacc.c  */
#line 1212 "src/cfgparser/oparse.y"
    {
  PARSER_DEBUG_PRINTF("Link quality aging factor %f\n", (yyvsp[(2) - (2)])->floating);
  olsr_cnf->lq_aging = (yyvsp[(2) - (2)])->floating;
  free((yyvsp[(2) - (2)]));
;}
    break;

  case 166:

/* Line 1455 of yacc.c  */
#line 1220 "src/cfgparser/oparse.y"
    {
  PARSER_DEBUG_PRINTF("Minimum TC validity time %f\n", (yyvsp[(2) - (2)])->floating);
  olsr_cnf->min_tc_vtime = (yyvsp[(2) - (2)])->floating;
  free((yyvsp[(2) - (2)]));
;}
    break;

  case 167:

/* Line 1455 of yacc.c  */
#line 1228 "src/cfgparser/oparse.y"
    {
  PARSER_DEBUG_PRINTF("Lock file %s\n", (yyvsp[(2) - (2)])->string);
  olsr_cnf->lock_file = (yyvsp[(2) - (2)])->string;
  free((yyvsp[(2) - (2)]));
;}
    break;

  case 168:

/* Line 1455 of yacc.c  */
#line 1235 "src/cfgparser/oparse.y"
    {
  olsr_cnf->lq_algorithm = (yyvsp[(2) - (2)])->string;
  PARSER_DEBUG_PRINTF("LQ Algorithm: %s\n", (yyvsp[(2) - (2)])->string);
  free((yyvsp[(2) - (2)]));
;}
    break;

  case 169:

/* Line 1455 of yacc.c  */
#line 1243 "src/cfgparser/oparse.y"
    {
  PARSER_DEBUG_PRINTF("NAT threshold %0.2f\n", (yyvsp[(2) - (2)])->floating);
  olsr_cnf->lq_nat_thresh = (yyvsp[(2) - (2)])->floating;
  free((yyvsp[(2) - (2)]));
;}
    break;

  case 170:

/* Line 1455 of yacc.c  */
#line 1251 "src/cfgparser/oparse.y"
    {
  PARSER_DEBUG_PRINTF("Clear screen %s\n", (yyvsp[(2) - (2)])->boolean ? "enabled" : "disabled");
  olsr_cnf->clear_screen = (yyvsp[(2) - (2)])->boolean;
  free((yyvsp[(2) - (2)]));
;}
    break;

  case 171:

/* Line 1455 of yacc.c  */
#line 1259 "src/cfgparser/oparse.y"
    {
  PARSER_DEBUG_PRINTF("Use NIIT ip translation: %s\n", (yyvsp[(2) - (2)])->boolean ? "enabled" : "disabled");
  olsr_cnf->use_niit = (yyvsp[(2) - (2)])->boolean;
  free((yyvsp[(2) - (2)]));
;}
    break;

  case 172:

/* Line 1455 of yacc.c  */
#line 1267 "src/cfgparser/oparse.y"
    {
	PARSER_DEBUG_PRINTF("Smart gateway system: %s\n", (yyvsp[(2) - (2)])->boolean ? "enabled" : "disabled");
	olsr_cnf->smart_gw_active = (yyvsp[(2) - (2)])->boolean;
	free((yyvsp[(2) - (2)]));
;}
    break;

  case 173:

/* Line 1455 of yacc.c  */
#line 1275 "src/cfgparser/oparse.y"
    {
	PARSER_DEBUG_PRINTF("Smart gateway allow client nat: %s\n", (yyvsp[(2) - (2)])->boolean ? "yes" : "no");
	olsr_cnf->smart_gw_allow_nat = (yyvsp[(2) - (2)])->boolean;
	free((yyvsp[(2) - (2)]));
;}
    break;

  case 174:

/* Line 1455 of yacc.c  */
#line 1283 "src/cfgparser/oparse.y"
    {
	PARSER_DEBUG_PRINTF("Smart gateway uplink: %s\n", (yyvsp[(2) - (2)])->string);
	if (strcasecmp((yyvsp[(2) - (2)])->string, GW_UPLINK_TXT[GW_UPLINK_NONE]) == 0) {
		olsr_cnf->smart_gw_type = GW_UPLINK_NONE;
	}
	if (strcasecmp((yyvsp[(2) - (2)])->string, GW_UPLINK_TXT[GW_UPLINK_IPV4]) == 0) {
		olsr_cnf->smart_gw_type = GW_UPLINK_IPV4;
	}
	else if (strcasecmp((yyvsp[(2) - (2)])->string, GW_UPLINK_TXT[GW_UPLINK_IPV6]) == 0) {
		olsr_cnf->smart_gw_type = GW_UPLINK_IPV6;
	}
	else if (strcasecmp((yyvsp[(2) - (2)])->string, GW_UPLINK_TXT[GW_UPLINK_IPV46]) == 0) {
		olsr_cnf->smart_gw_type = GW_UPLINK_IPV46;
	}
	else {
		fprintf(stderr, "Bad gateway uplink type: %s\n", (yyvsp[(2) - (2)])->string);
		YYABORT;
	}
	free((yyvsp[(2) - (2)]));
;}
    break;

  case 175:

/* Line 1455 of yacc.c  */
#line 1306 "src/cfgparser/oparse.y"
    {
	PARSER_DEBUG_PRINTF("Smart gateway speed: %u uplink/%u downlink kbit/s\n", (yyvsp[(2) - (3)])->integer, (yyvsp[(3) - (3)])->integer);
	olsr_cnf->smart_gw_uplink = (yyvsp[(2) - (3)])->integer;
	olsr_cnf->smart_gw_downlink = (yyvsp[(3) - (3)])->integer;
	free((yyvsp[(2) - (3)]));
	free((yyvsp[(3) - (3)]));
;}
    break;

  case 176:

/* Line 1455 of yacc.c  */
#line 1316 "src/cfgparser/oparse.y"
    {
	PARSER_DEBUG_PRINTF("Smart gateway uplink nat: %s\n", (yyvsp[(2) - (2)])->boolean ? "yes" : "no");
	olsr_cnf->smart_gw_uplink_nat = (yyvsp[(2) - (2)])->boolean;
	free((yyvsp[(2) - (2)]));
;}
    break;

  case 177:

/* Line 1455 of yacc.c  */
#line 1324 "src/cfgparser/oparse.y"
    {
  PARSER_DEBUG_PRINTF("Smart gateway prefix: %s %u\n", (yyvsp[(2) - (3)])->string, (yyvsp[(3) - (3)])->integer);
	if (inet_pton(olsr_cnf->ip_version, (yyvsp[(2) - (3)])->string, &olsr_cnf->smart_gw_prefix.prefix) == 0) {
	  fprintf(stderr, "Bad IP part of gateway prefix: %s\n", (yyvsp[(2) - (3)])->string);
    YYABORT;
  }
	olsr_cnf->smart_gw_prefix.prefix_len = (uint8_t)(yyvsp[(3) - (3)])->integer;
	
	free((yyvsp[(2) - (3)]));
	free((yyvsp[(3) - (3)]));
;}
    break;

  case 178:

/* Line 1455 of yacc.c  */
#line 1336 "src/cfgparser/oparse.y"
    {
	PARSER_DEBUG_PRINTF("Smart gateway prefix: %s %u\n", (yyvsp[(2) - (4)])->string, (yyvsp[(4) - (4)])->integer);
	if (inet_pton(olsr_cnf->ip_version, (yyvsp[(2) - (4)])->string, &olsr_cnf->smart_gw_prefix.prefix) == 0) {
	  fprintf(stderr, "Bad IP part of gateway prefix: %s\n", (yyvsp[(2) - (4)])->string);
    YYABORT;
  }
	olsr_cnf->smart_gw_prefix.prefix_len = (uint8_t)(yyvsp[(4) - (4)])->integer;
	
	free((yyvsp[(2) - (4)]));
	free((yyvsp[(4) - (4)]));
;}
    break;

  case 179:

/* Line 1455 of yacc.c  */
#line 1350 "src/cfgparser/oparse.y"
    {
	PARSER_DEBUG_PRINTF("Use originator for routes src-ip: %s\n", (yyvsp[(2) - (2)])->boolean ? "yes" : "no");
	olsr_cnf->use_src_ip_routes = (yyvsp[(2) - (2)])->boolean;
	free((yyvsp[(2) - (2)]));
;}
    break;

  case 180:

/* Line 1455 of yacc.c  */
#line 1358 "src/cfgparser/oparse.y"
    {
  PARSER_DEBUG_PRINTF("Fixed Main IP: %s\n", (yyvsp[(2) - (2)])->string);
  
  if (olsr_cnf->ip_version != AF_INET
      || inet_pton(olsr_cnf->ip_version, (yyvsp[(2) - (2)])->string, &olsr_cnf->main_addr) != 1) {
    fprintf(stderr, "Bad main IP: %s\n", (yyvsp[(2) - (2)])->string);
    YYABORT;
  }
  free((yyvsp[(2) - (2)]));
;}
    break;

  case 181:

/* Line 1455 of yacc.c  */
#line 1369 "src/cfgparser/oparse.y"
    {
  PARSER_DEBUG_PRINTF("Fixed Main IP: %s\n", (yyvsp[(2) - (2)])->string);
  
  if (olsr_cnf->ip_version != AF_INET6
      || inet_pton(olsr_cnf->ip_version, (yyvsp[(2) - (2)])->string, &olsr_cnf->main_addr) != 1) {
    fprintf(stderr, "Bad main IP: %s\n", (yyvsp[(2) - (2)])->string);
    YYABORT;
  }
  free((yyvsp[(2) - (2)]));
;}
    break;

  case 182:

/* Line 1455 of yacc.c  */
#line 1381 "src/cfgparser/oparse.y"
    {
  struct plugin_entry *pe, *last;
  
  pe = olsr_cnf->plugins;
  last = NULL;
  while (pe != NULL) {
    if (strcmp(pe->name, (yyvsp[(2) - (2)])->string) == 0) {
      free ((yyvsp[(2) - (2)])->string);
      break;
    }
    last = pe;
    pe = pe->next;
  }

  if (pe != NULL) {
    /* remove old plugin from list to add it later at the beginning */
    if (last) {
      last->next = pe->next;
    }
    else {
      olsr_cnf->plugins = pe->next;
    }
  }
  else {
    pe = malloc(sizeof(*pe));

    if (pe == NULL) {
      fprintf(stderr, "Out of memory(ADD PL)\n");
      YYABORT;
    }

    pe->name = (yyvsp[(2) - (2)])->string;
    pe->params = NULL;

    PARSER_DEBUG_PRINTF("Plugin: %s\n", (yyvsp[(2) - (2)])->string);
  }
  
  /* Queue */
  pe->next = olsr_cnf->plugins;
  olsr_cnf->plugins = pe;

  free((yyvsp[(2) - (2)]));
;}
    break;

  case 183:

/* Line 1455 of yacc.c  */
#line 1427 "src/cfgparser/oparse.y"
    {
  struct plugin_param *pp = malloc(sizeof(*pp));
  
  if (pp == NULL) {
    fprintf(stderr, "Out of memory(ADD PP)\n");
    YYABORT;
  }
  
  PARSER_DEBUG_PRINTF("Plugin param key:\"%s\" val: \"%s\"\n", (yyvsp[(2) - (3)])->string, (yyvsp[(3) - (3)])->string);
  
  pp->key = (yyvsp[(2) - (3)])->string;
  pp->value = (yyvsp[(3) - (3)])->string;

  /* Queue */
  pp->next = olsr_cnf->plugins->params;
  olsr_cnf->plugins->params = pp;

  free((yyvsp[(2) - (3)]));
  free((yyvsp[(3) - (3)]));
;}
    break;

  case 184:

/* Line 1455 of yacc.c  */
#line 1450 "src/cfgparser/oparse.y"
    {
    //PARSER_DEBUG_PRINTF("Comment\n");
;}
    break;



/* Line 1455 of yacc.c  */
#line 3225 "src/cfgparser/oparse.c"
      default: break;
    }
  YY_SYMBOL_PRINT ("-> $$ =", yyr1[yyn], &yyval, &yyloc);

  YYPOPSTACK (yylen);
  yylen = 0;
  YY_STACK_PRINT (yyss, yyssp);

  *++yyvsp = yyval;

  /* Now `shift' the result of the reduction.  Determine what state
     that goes to, based on the state we popped back to and the rule
     number reduced by.  */

  yyn = yyr1[yyn];

  yystate = yypgoto[yyn - YYNTOKENS] + *yyssp;
  if (0 <= yystate && yystate <= YYLAST && yycheck[yystate] == *yyssp)
    yystate = yytable[yystate];
  else
    yystate = yydefgoto[yyn - YYNTOKENS];

  goto yynewstate;


/*------------------------------------.
| yyerrlab -- here on detecting error |
`------------------------------------*/
yyerrlab:
  /* If not already recovering from an error, report this error.  */
  if (!yyerrstatus)
    {
      ++yynerrs;
#if ! YYERROR_VERBOSE
      yyerror (YY_("syntax error"));
#else
      {
	YYSIZE_T yysize = yysyntax_error (0, yystate, yychar);
	if (yymsg_alloc < yysize && yymsg_alloc < YYSTACK_ALLOC_MAXIMUM)
	  {
	    YYSIZE_T yyalloc = 2 * yysize;
	    if (! (yysize <= yyalloc && yyalloc <= YYSTACK_ALLOC_MAXIMUM))
	      yyalloc = YYSTACK_ALLOC_MAXIMUM;
	    if (yymsg != yymsgbuf)
	      YYSTACK_FREE (yymsg);
	    yymsg = (char *) YYSTACK_ALLOC (yyalloc);
	    if (yymsg)
	      yymsg_alloc = yyalloc;
	    else
	      {
		yymsg = yymsgbuf;
		yymsg_alloc = sizeof yymsgbuf;
	      }
	  }

	if (0 < yysize && yysize <= yymsg_alloc)
	  {
	    (void) yysyntax_error (yymsg, yystate, yychar);
	    yyerror (yymsg);
	  }
	else
	  {
	    yyerror (YY_("syntax error"));
	    if (yysize != 0)
	      goto yyexhaustedlab;
	  }
      }
#endif
    }



  if (yyerrstatus == 3)
    {
      /* If just tried and failed to reuse lookahead token after an
	 error, discard it.  */

      if (yychar <= YYEOF)
	{
	  /* Return failure if at end of input.  */
	  if (yychar == YYEOF)
	    YYABORT;
	}
      else
	{
	  yydestruct ("Error: discarding",
		      yytoken, &yylval);
	  yychar = YYEMPTY;
	}
    }

  /* Else will try to reuse lookahead token after shifting the error
     token.  */
  goto yyerrlab1;


/*---------------------------------------------------.
| yyerrorlab -- error raised explicitly by YYERROR.  |
`---------------------------------------------------*/
yyerrorlab:

  /* Pacify compilers like GCC when the user code never invokes
     YYERROR and the label yyerrorlab therefore never appears in user
     code.  */
  if (/*CONSTCOND*/ 0)
     goto yyerrorlab;

  /* Do not reclaim the symbols of the rule which action triggered
     this YYERROR.  */
  YYPOPSTACK (yylen);
  yylen = 0;
  YY_STACK_PRINT (yyss, yyssp);
  yystate = *yyssp;
  goto yyerrlab1;


/*-------------------------------------------------------------.
| yyerrlab1 -- common code for both syntax error and YYERROR.  |
`-------------------------------------------------------------*/
yyerrlab1:
  yyerrstatus = 3;	/* Each real token shifted decrements this.  */

  for (;;)
    {
      yyn = yypact[yystate];
      if (yyn != YYPACT_NINF)
	{
	  yyn += YYTERROR;
	  if (0 <= yyn && yyn <= YYLAST && yycheck[yyn] == YYTERROR)
	    {
	      yyn = yytable[yyn];
	      if (0 < yyn)
		break;
	    }
	}

      /* Pop the current state because it cannot handle the error token.  */
      if (yyssp == yyss)
	YYABORT;


      yydestruct ("Error: popping",
		  yystos[yystate], yyvsp);
      YYPOPSTACK (1);
      yystate = *yyssp;
      YY_STACK_PRINT (yyss, yyssp);
    }

  *++yyvsp = yylval;


  /* Shift the error token.  */
  YY_SYMBOL_PRINT ("Shifting", yystos[yyn], yyvsp, yylsp);

  yystate = yyn;
  goto yynewstate;


/*-------------------------------------.
| yyacceptlab -- YYACCEPT comes here.  |
`-------------------------------------*/
yyacceptlab:
  yyresult = 0;
  goto yyreturn;

/*-----------------------------------.
| yyabortlab -- YYABORT comes here.  |
`-----------------------------------*/
yyabortlab:
  yyresult = 1;
  goto yyreturn;

#if !defined(yyoverflow) || YYERROR_VERBOSE
/*-------------------------------------------------.
| yyexhaustedlab -- memory exhaustion comes here.  |
`-------------------------------------------------*/
yyexhaustedlab:
  yyerror (YY_("memory exhausted"));
  yyresult = 2;
  /* Fall through.  */
#endif

yyreturn:
  if (yychar != YYEMPTY)
     yydestruct ("Cleanup: discarding lookahead",
		 yytoken, &yylval);
  /* Do not reclaim the symbols of the rule which action triggered
     this YYABORT or YYACCEPT.  */
  YYPOPSTACK (yylen);
  YY_STACK_PRINT (yyss, yyssp);
  while (yyssp != yyss)
    {
      yydestruct ("Cleanup: popping",
		  yystos[*yyssp], yyvsp);
      YYPOPSTACK (1);
    }
#ifndef yyoverflow
  if (yyss != yyssa)
    YYSTACK_FREE (yyss);
#endif
#if YYERROR_VERBOSE
  if (yymsg != yymsgbuf)
    YYSTACK_FREE (yymsg);
#endif
  /* Make sure YYID is used.  */
  return YYID (yyresult);
}



/* Line 1675 of yacc.c  */
#line 1457 "src/cfgparser/oparse.y"


void yyerror (const char *string)
{
  fprintf(stderr, "Config line %d: %s\n", current_line, string);
}

