set(OPENMW_SOURCES
    engine.cpp
    options.cpp
)

if(VITA)
    list(APPEND OPENMW_SOURCES
        vita/VitaInit.cpp
        vita/VitaIme.cpp
        vita/VitaMemAudit.cpp
        vita/VitaSimWorker.cpp
        vita/VitaGLWorker.cpp
    )
endif()

set(OPENMW_RESOURCES
    ${CMAKE_SOURCE_DIR}/files/windows/openmw.rc
    ${CMAKE_SOURCE_DIR}/files/windows/openmw.exe.manifest
)

set(OPENMW_HEADERS
    doc.hpp
    engine.hpp
    options.hpp
    profile.hpp
)

source_group(apps/openmw FILES main.cpp androidmain.cpp ${OPENMW_SOURCES} ${OPENMW_HEADERS} ${OPENMW_RESOURCES})

add_openmw_dir (mwrender
    actors objects renderingmanager animation rotatecontroller sky skyutil npcanimation esm4npcanimation vismask
    creatureanimation effectmanager util renderinginterface pathgrid rendermode weaponanimation screenshotmanager
    bulletdebugdraw globalmap characterpreview camera localmap water terrainstorage ripplesimulation
    renderbin actoranimation landmanager navmesh actorspaths recastmesh fogmanager objectpaging groundcover
    postprocessor pingpongcull luminancecalculator pingpongcanvas transparentpass precipitationocclusion ripples
    actorutil distortion animationpriority bonegroup blendmask animblendcontroller
    )

add_openmw_dir (mwinput
    actions actionmanager bindingsmanager controllermanager controlswitch
    inputmanagerimp mousemanager keyboardmanager sensormanager gyromanager
    vitatouchzones
    )

add_openmw_dir (mwgui
    layout textinput widgets race class birth review windowmanagerimp console dialogue
    windowbase statswindow messagebox journalwindow charactercreation
    mapwindow windowpinnablebase tooltips scrollwindow bookwindow resourceskin
    formatting inventorywindow container hud countdialog tradewindow settingswindow
    confirmationdialog alchemywindow referenceinterface spellwindow mainmenu quickkeysmenu
    itemselection spellbuyingwindow loadingscreen levelupdialog waitdialog spellcreationdialog
    enchantingdialog trainingwindow travelwindow exposedwindow cursor spellicons
    merchantrepair repair soulgemdialog companionwindow bookpage journalviewmodel journalbooks
    itemmodel containeritemmodel inventoryitemmodel sortfilteritemmodel itemview
    tradeitemmodel companionitemmodel pickpocketitemmodel controllers savegamedialog
    recharge mode videowidget backgroundimage itemwidget screenfader debugwindow spellmodel spellview
    draganddrop timeadvancer jailscreen itemchargeview keyboardnavigation textcolours statswatcher
    postprocessorhud settings worlditemmodel itemtransfer controllerbuttonsoverlay inventorytabsoverlay vitatouchoverlay
    )

add_openmw_dir (mwdialogue
    dialoguemanagerimp journalimp journalentry quest topic filter selectwrapper hypertextparser keywordsearch scripttest
    )

add_openmw_dir (mwscript
    locals scriptmanagerimp compilercontext interpretercontext cellextensions miscextensions
    guiextensions soundextensions skyextensions statsextensions containerextensions
    aiextensions controlextensions extensions globalscripts ref dialogueextensions
    animationextensions transformationextensions consoleextensions userextensions
    )

add_openmw_dir (mwlua
    luamanagerimp object objectlists userdataserializer luaevents engineevents objectvariant
    context menuscripts globalscripts localscripts playerscripts luabindings objectbindings cellbindings coremwscriptbindings
    mwscriptbindings camerabindings vfsbindings uibindings soundbindings inputbindings nearbybindings dialoguebindings
    postprocessingbindings stats recordstore debugbindings corebindings worldbindings worker landbindings magicbindings factionbindings
    classbindings itemdata inputprocessor animationbindings birthsignbindings racebindings markupbindings weatherbindings regionbindings
    types/types types/door types/item types/actor types/container types/lockable types/weapon types/npc
    types/creature types/player types/activator types/book types/lockpick types/probe types/apparatus
    types/potion types/ingredient types/misc types/repair types/armor types/light types/static
    types/clothing types/levelledlist types/terminal
    )

add_openmw_dir (mwsound
    soundmanagerimp openaloutput ffmpegdecoder sound soundbuffer sounddecoder soundoutput
    loudness movieaudiofactory alext efx efxpresets regionsoundselector watersoundupdater
    )

add_openmw_dir (mwworld
    refdata worldimp scene globals class action nullaction actionteleport
    containerstore actiontalk actiontake manualref player cellvisitors failedaction
    worldmodel localscripts customdata inventorystore ptr actionopen actionread actionharvest
    actionequip timestamp actionalchemy cellstore actionapply actioneat
    store esmstore fallback actionrepair actionsoulgem livecellref actiondoor
    contentloader esmloader actiontrap cellreflist cellref weather projectilemanager
    cellpreloader datetimemanager groundcoverstore magiceffects cell ptrregistry
    positioncellgrid
    )

add_openmw_dir (mwphysics
    physicssystem trace collisiontype actor convert object heightfield closestnotmerayresultcallback
    contacttestresultcallback stepper movementsolver projectile
    actorconvexcallback raycasting mtphysics contacttestwrapper projectileconvexcallback
    )

add_openmw_dir (mwclass
    classes activator creature npc weapon armor potion apparatus book clothing container door
    ingredient creaturelevlist itemlevlist light lockpick misc probe repair static actor bodypart
    esm4base esm4npc light4
    )

add_openmw_dir (mwmechanics
    mechanicsmanagerimp stat creaturestats magiceffects movement actorutil spelllist
    drawstate spells activespells npcstats aipackage aisequence aipursue alchemy aiwander aitravel aifollow aiavoiddoor aibreathe
    aicast aiescort aiface aiactivate aicombat recharge repair enchanting pathfinding pathgrid security spellcasting spellresistance
    disease pickpocket levelledlist combat steering obstacle autocalcspell difficultyscaling aicombataction summoning
    character actors objects aistate weaponpriority spellpriority weapontype spellutil
    spelleffects
    )

add_openmw_dir (mwstate
    statemanagerimp charactermanager character quicksavemanager
    )

add_openmw_dir (mwbase
    environment world scriptmanager dialoguemanager journal soundmanager mechanicsmanager
    inputmanager windowmanager statemanager luamanager
    )

# Main executable

add_library(openmw-lib STATIC
    ${OPENMW_FILES}
    ${OPENMW_SOURCES}
)

if (MSVC)
    # these TUs take a lot of memory for codegen
    # in CI, they often go over the 8GB memory limit if another cl.exe is running at the same time
    # this option (make sure you're not looking at the docs for the identically-named linker option) stops four codegen threads being spun off
    # it hurts build time ~33% to enable this unnecessarily, though, so target it to the hungriest boys
    set_source_files_properties(
        mwlua/dialoguebindings.cpp
        mwlua/magicbindings.cpp
        mwlua/objectbindings
        mwlua/stats.cpp
        mwworld/esmstore.cpp
        PROPERTIES
        COMPILE_OPTIONS /cgthreads1
    )
endif()

if (VITA)
    set_source_files_properties(
        mwgui/console.cpp
        mwgui/debugwindow.cpp
        mwgui/settingswindow.cpp
        mwgui/savegamedialog.cpp
        mwgui/journalwindow.cpp
        mwgui/journalbooks.cpp
        mwgui/journalviewmodel.cpp
        mwgui/charactercreation.cpp
        mwgui/race.cpp
        mwgui/class.cpp
        mwgui/birth.cpp
        mwgui/review.cpp
        mwgui/levelupdialog.cpp
        mwgui/spellcreationdialog.cpp
        mwgui/enchantingdialog.cpp
        mwgui/trainingwindow.cpp
        mwgui/travelwindow.cpp
        mwgui/merchantrepair.cpp
        mwgui/repair.cpp
        mwgui/recharge.cpp
        mwgui/postprocessorhud.cpp
        mwgui/textinput.cpp
        mwgui/itemselection.cpp
        mwgui/companionwindow.cpp
        mwgui/soulgemdialog.cpp
        mwgui/waitdialog.cpp
        mwgui/jailscreen.cpp
        PROPERTIES
        COMPILE_FLAGS "-Os"
    )
endif()

if(BUILD_OPENMW)
    if (ANDROID)
        add_library(openmw SHARED
            main.cpp
            androidmain.cpp
        )
    else()
        openmw_add_executable(openmw
            ${APPLE_BUNDLE_RESOURCES}
            ${OPENMW_RESOURCES}
            main.cpp
        )
    endif()

    target_link_libraries(openmw openmw-lib)

    # Workaround necessary to ensure osgAnimation::MatrixLinearSampler dynamic casts work under Clang
    # NOTE: it's unclear whether the broken behavior is spec-compliant
    if (CMAKE_CXX_COMPILER_ID STREQUAL Clang)
        set_target_properties(openmw PROPERTIES ENABLE_EXPORTS ON)
    endif()

    # vita_create_self expects the ELF in CMAKE_CURRENT_BINARY_DIR
    if (VITA)
        set_target_properties(openmw PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
        )
        # FFmpeg TX stubs in executable preempt libavutil.a (saves 16MB BSS).
        target_sources(openmw PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/vita/ffmpeg_tx_stubs.c
        )
    endif()
endif()

# Sound stuff - here so CMake doesn't stupidly recompile EVERYTHING
# when we change the backend.
include_directories(
    ${FFmpeg_INCLUDE_DIRS}
)

target_link_libraries(openmw-lib
    # CMake's built-in OSG finder does not use pkgconfig, so we have to
    # manually ensure the order is correct for inter-library dependencies.
    # This only makes a difference with `-DOPENMW_USE_SYSTEM_OSG=ON -DOSG_STATIC=ON`.
    # https://gitlab.kitware.com/cmake/cmake/-/issues/21701
    ${OSGPARTICLE_LIBRARIES}
    ${OSGVIEWER_LIBRARIES}
    ${OSGGA_LIBRARIES}
    ${OSGSHADOW_LIBRARIES}
    ${OSGDB_LIBRARIES}
    ${OSGUTIL_LIBRARIES}
    ${OSG_LIBRARIES}
    Boost::program_options
    ${OPENAL_LIBRARY}
    ${FFmpeg_LIBRARIES}
    ${MyGUI_LIBRARIES}
    SDL2::SDL2
    ${RecastNavigation_LIBRARIES}
    osg-ffmpeg-videoplayer
    oics
    components
)

if (MSVC AND PRECOMPILE_HEADERS_WITH_MSVC)
    target_precompile_headers(openmw-lib PRIVATE
        <boost/program_options/options_description.hpp>

        <sol/sol.hpp>

        <osg/StateSet>
        <osg/Node>
        <osg/Drawable>
        <osg/Camera>

        <osgViewer/Viewer>

        <MyGUI_Widget.h>

        <algorithm>
        <filesystem>
        <fstream>
        <functional>
        <memory>
        <string>
        <string_view>
        <vector>
    )
endif()

if (ANDROID)
    target_link_libraries(openmw-lib EGL android log z)
endif (ANDROID)

if (VITA)
    # Attr-less pthreads get 256KB stacks (default 32KB; Bullet EPA needs ~30KB).
    target_link_options(openmw PRIVATE "-Wl,--wrap=pthread_create")
    # OSG libs excluded from USED_OSG_COMPONENTS but still needed for linking.
    foreach(_osgextra osgParticle osgShadow)
        if(NOT TARGET ${_osgextra})
            add_library(${_osgextra} STATIC IMPORTED)
            set_target_properties(${_osgextra} PROPERTIES
                IMPORTED_LOCATION "${CMAKE_BINARY_DIR}/_deps/osg-build/lib/lib${_osgextra}.a")
            add_dependencies(${_osgextra} osg)
        endif()
    endforeach()
    target_link_libraries(openmw-lib osgParticle osgShadow)
    # FFmpeg's libavcodec links libmp3lame for MP3 encoding support
    target_link_libraries(openmw-lib mp3lame)
    # Vita system libraries needed by vitaGL
    target_link_libraries(openmw-lib
        vitashark
        mathneon
        SceShaccCgExt
        SceShaccCg_stub
        SceKernelDmacMgr_stub
        SceSblSsMgr_stub
        taihen_stub
        SceKernelModulemgr_stub
        SceSysmem_stub
        SceVshBridge_stub
    )
    # OpenAL audio backend needs SCE audio port functions
    target_link_libraries(openmw-lib
        SceAudio_stub
    )
    # dlopen/dlclose/dlsym stubs (osgDB::DynamicLibrary, LuaJIT)
    target_link_libraries(openmw-lib dl)
    # PTE (pthreads-to-eabi) — needed by OpenThreads for cleanup handlers
    target_link_libraries(openmw-lib pthread)
    # pthread_cancel=0 removed: broke real std::mutex init for game threads.
endif()

if (USE_SYSTEM_TINYXML)
    target_link_libraries(openmw-lib ${TinyXML_LIBRARIES})
endif()

if (NOT UNIX)
    target_link_libraries(openmw-lib ${SDL2MAIN_LIBRARY})
endif()

# Fix for not visible pthreads functions for linker with glibc 2.15
if (UNIX AND NOT APPLE)
    target_link_libraries(openmw-lib ${CMAKE_THREAD_LIBS_INIT})
endif()

if(APPLE AND BUILD_OPENMW)
    set(BUNDLE_RESOURCES_DIR "${APP_BUNDLE_DIR}/Contents/Resources")

    set(OPENMW_RESOURCES_ROOT ${BUNDLE_RESOURCES_DIR})

    add_subdirectory(../../files/ ${CMAKE_CURRENT_BINARY_DIR}/files)

    configure_file("${OpenMW_BINARY_DIR}/defaults.bin" ${BUNDLE_RESOURCES_DIR} COPYONLY)
    configure_file("${OpenMW_BINARY_DIR}/openmw.cfg" ${BUNDLE_RESOURCES_DIR} COPYONLY)
    configure_file("${OpenMW_BINARY_DIR}/gamecontrollerdb.txt" ${BUNDLE_RESOURCES_DIR} COPYONLY)

    add_custom_command(TARGET openmw
        POST_BUILD
        COMMAND cp "${OpenMW_BINARY_DIR}/resources/version" "${BUNDLE_RESOURCES_DIR}/resources")

    find_library(COCOA_FRAMEWORK Cocoa)
    find_library(IOKIT_FRAMEWORK IOKit)
    target_link_libraries(openmw ${COCOA_FRAMEWORK} ${IOKIT_FRAMEWORK})

    if (FFmpeg_FOUND)
        target_link_options(openmw PRIVATE "LINKER:SHELL:-framework CoreVideo"
                                           "LINKER:SHELL:-framework CoreMedia"
                                           "LINKER:SHELL:-framework VideoToolbox"
                                           "LINKER:SHELL:-framework AudioToolbox"
                                           "LINKER:SHELL:-framework VideoDecodeAcceleration")
    endif()
endif()

if (BUILD_WITH_CODE_COVERAGE)
    target_compile_options(openmw-lib PRIVATE --coverage)
    target_link_libraries(openmw-lib gcov)
    if (NOT ANDROID AND BUILD_OPENMW)
        target_compile_options(openmw PRIVATE --coverage)
        target_link_libraries(openmw gcov)
    endif()
endif()

if (WIN32 AND BUILD_OPENMW)
    INSTALL(TARGETS openmw RUNTIME DESTINATION ".")
endif()

if (VITA AND BUILD_OPENMW)
    include("${VITASDK}/share/vita.cmake" REQUIRED)

    if(NOT VITA_TITLEID)
        set(VITA_TITLEID "OMWV00001")
    endif()
    if(NOT VITA_APP_NAME)
        set(VITA_APP_NAME "OpenMW")
    endif()

    # Extra memory mode: +109MB RAM (Vita disables background services)
    set(VITA_MKSFOEX_FLAGS "${VITA_MKSFOEX_FLAGS} -d ATTRIBUTE2=12")

    # Bundle MOP + Project Atlas mod assets into the VPK as Morrowind BSA
    # archives at app0:/resources/baked-mods/<NN>-<slug>.bsa. The cfg-side
    # `data=` + `fallback-archive=` entries are written at boot by
    # autoDetectBakedMods() in apps/openmw/vita/VitaInit.cpp, which scans
    # the BSAs present in app0 — so the cfg side needs no build-time wiring.
    # Source assets live in <repo>/mods/; if that tree is absent the bake
    # script no-ops cleanly.
    option(OPENMW_VITA_BAKE_MODS
        "Bundle MOP + Project Atlas mod assets into the VPK as BSAs" ON)

    # Fix ARM relocations and veneer literal pools BEFORE conversion.
    add_custom_command(TARGET openmw POST_BUILD
        COMMAND python3 ${CMAKE_SOURCE_DIR}/scripts/vita/fix-relocations.py
            ${CMAKE_CURRENT_BINARY_DIR}/openmw
        COMMAND python3 ${CMAKE_SOURCE_DIR}/scripts/vita/fix-veneers.py
            ${CMAKE_CURRENT_BINARY_DIR}/openmw
        COMMENT "Fixing ARM relocations and veneer literal pools"
        VERBATIM
    )

    # Convert ELF to Vita SELF (eboot.bin)
    vita_create_self(eboot.bin openmw
        UNSAFE
        NOASLR
        ATTRIBUTE 0x02000000
    )

    # Zero BSS: Vita loader doesn't zero memsz > filesz gap.
    add_custom_command(TARGET openmw-velf POST_BUILD
        COMMAND python3 ${CMAKE_SOURCE_DIR}/scripts/vita/fix-bss.py
            ${CMAKE_CURRENT_BINARY_DIR}/openmw.velf
        COMMENT "Expanding BSS in VELF (zeroing uninitialized data)"
    )
    # eboot.bin-self depends on openmw.velf (file), but file-level deps bypass
    # POST_BUILD. Force target-level dep so fix-bss.py finishes before
    # vita-make-fself reads the VELF. Without this, -jN causes a read race.
    add_dependencies(eboot.bin-self openmw-velf)

    # LiveArea assets
    set(VITA_DATA_DIR ${CMAKE_SOURCE_DIR}/data/openMW)

    # Create VPK package
    vita_create_vpk(openmw.vpk ${VITA_TITLEID} eboot.bin
        VERSION 02.20
        NAME ${VITA_APP_NAME}
        FILE ${CMAKE_SOURCE_DIR}/apps/openmw/vita/openmw.cfg openmw.cfg
        FILE ${CMAKE_BINARY_DIR}/defaults.bin defaults.bin
        FILE ${CMAKE_SOURCE_DIR}/apps/openmw/vita/settings.cfg settings.cfg
        FILE ${VITA_DATA_DIR}/sce_sys/icon0.png sce_sys/icon0.png
        FILE ${VITA_DATA_DIR}/sce_sys/livearea/contents/startup.png sce_sys/livearea/contents/startup.png
        FILE ${VITA_DATA_DIR}/sce_sys/livearea/contents/bg0.png sce_sys/livearea/contents/bg0.png
        FILE ${VITA_DATA_DIR}/sce_sys/livearea/contents/bg.png sce_sys/livearea/contents/bg.png
        FILE ${VITA_DATA_DIR}/sce_sys/livearea/contents/template.xml sce_sys/livearea/contents/template.xml
        # Pre-warmed VFS file index for path-stable engine resource dirs
        # (app0:/resources/vfs and app0:/resources/vfs-mw). Skips ~22KB of
        # recursive_directory_iterator work on first launch. Copied to
        # ux0:data/openmw/config/ at boot if not already present.
        # PC-cooked warm packages (fallback when no learned copy on ux0)
        FILE ${CMAKE_SOURCE_DIR}/apps/openmw/vita/warm/commonwarm.txt warm/commonwarm.txt
        FILE ${CMAKE_SOURCE_DIR}/apps/openmw/vita/warm/warm_regions.txt warm/warm_regions.txt
        FILE ${CMAKE_SOURCE_DIR}/apps/openmw/vita/warm/object_index.txt warm/object_index.txt
        FILE ${CMAKE_SOURCE_DIR}/apps/openmw/vita/vfs_seed/vfs_dir_0.bin vfs_seed/vfs_dir_0.bin
        FILE ${CMAKE_SOURCE_DIR}/apps/openmw/vita/vfs_seed/vfs_dir_1.bin vfs_seed/vfs_dir_1.bin
        # GOG ships ~21k loose files extracted from the BSAs, making the
        # first-boot recursive scan effectively forever. Copied into vfs_dir_2.bin
        # only when a GOG marker (bcsounds.esp) is detected at the data root.
        # Generated by scripts/vita/build-vfs-seed.py from a clean GOG dump.
        FILE ${CMAKE_SOURCE_DIR}/apps/openmw/vita/vfs_seed/vfs_dir_2_gog.bin vfs_seed/vfs_dir_2_gog.bin
    )

    # Inject ${CMAKE_BINARY_DIR}/resources (shaders, fonts, lua_api, vfs, ...)
    # into the VPK. vita_create_vpk only packs explicit FILE entries; OSG/OpenMW
    # needs the full resources tree or launch crashes with missing shader files.
    add_custom_command(TARGET openmw.vpk-vpk POST_BUILD
        COMMAND python3 ${CMAKE_SOURCE_DIR}/scripts/vita/add-resources-to-vpk.py
            ${CMAKE_CURRENT_BINARY_DIR}/openmw.vpk
            ${CMAKE_BINARY_DIR}/resources
        COMMENT "Adding resources/ directory to VPK"
    )

    # Bundle MOP + Project Atlas mod assets at resources/baked-mods/<name>/.
    # Script no-ops cleanly if <repo>/mods/ or individual sources are missing,
    # so this is safe to leave enabled on dev machines without the mods tree.
    if(OPENMW_VITA_BAKE_MODS)
        add_custom_command(TARGET openmw.vpk-vpk POST_BUILD
            COMMAND python3
                ${CMAKE_SOURCE_DIR}/scripts/vita/add-baked-mods-to-vpk.py
                ${CMAKE_CURRENT_BINARY_DIR}/openmw.vpk
                ${CMAKE_SOURCE_DIR}/mods
            COMMENT "Adding baked-in optimization mods (MOP + Project Atlas) to VPK"
        )
    endif()
endif()
